#include "hknpcollision.h"

#include "gl/hknplayout.h"
#include "gl/hkpackfile.h"

#include <QDebug>
#include <QHash>

#include <cmath>

//! @file hknpcollision.cpp Wireframes of Fallout 4 collision, and the cache of parsed physics systems

/*
 * Fallout 4 stores collision as a complete Havok packfile inside bhkPhysicsSystem (read by HkPackfile, class
 * layouts in hknplayout.h). This file turns each body's shape into a line list for the viewport. Only the shape
 * types needed for drawing are decoded; anything unknown is skipped.
 */

using namespace HknpLayout;

namespace
{

//! An affine transform in Havok's convention: v' = x * v[0] + y * v[1] + z * v[2] + t
struct Affine
{
	Vector3 x = Vector3( 1.0f, 0.0f, 0.0f );
	Vector3 y = Vector3( 0.0f, 1.0f, 0.0f );
	Vector3 z = Vector3( 0.0f, 0.0f, 1.0f );
	Vector3 t;

	Vector3 operator*( const Vector3 & v ) const { return x * v[0] + y * v[1] + z * v[2] + t; }
	Affine operator*( const Affine & b ) const
	{
		Affine r;
		r.x = x * b.x[0] + y * b.x[1] + z * b.x[2];
		r.y = x * b.y[0] + y * b.y[1] + z * b.y[2];
		r.z = x * b.z[0] + y * b.z[1] + z * b.z[2];
		r.t = *this * b.t;
		return r;
	}
};

//! Work limits for one physics system, shared by all of its bodies, so a malformed file cannot freeze the viewer.
//! The largest vanilla collision (precombined *_Physics.nif compressed meshes, ~600k triangles) needs about
//! 4.5M line points, far below these limits.
struct Budget
{
	qsizetype linePoints = 32 * 1024 * 1024;
	qsizetype shapeVisits = 1024 * 1024;
};

//! Builds the wireframe (line list) of one body's shape
class ShapeBuilder
{
public:
	ShapeBuilder( const HkPackfile & p, QVector<Vector3> & l, Budget & b ) : pf( p ), lines( l ), budget( b ) {}

	void addShape( quint32 shape, const Affine & xf );

private:
	static constexpr int circleSegments = 24;
	static constexpr int maxDepth = 16;

	void addShapeContents( quint32 shape, const Affine & xf );

	void line( const Affine & xf, const Vector3 & a, const Vector3 & b )
	{
		if ( budget.linePoints < 2 )
			return;
		budget.linePoints -= 2;
		lines.append( xf * a );
		lines.append( xf * b );
	}
	void arc( const Affine & xf, const Vector3 & c, const Vector3 & u, const Vector3 & v, float a0, float a1, int segments )
	{
		Vector3 prev = c + u * std::cos( a0 ) + v * std::sin( a0 );
		for ( int i = 1; i <= segments; i++ ) {
			float a = a0 + ( a1 - a0 ) * float( i ) / float( segments );
			Vector3 p = c + u * std::cos( a ) + v * std::sin( a );
			line( xf, prev, p );
			prev = p;
		}
	}
	void sphere( const Affine & xf, const Vector3 & c, float r );
	void capsule( const Affine & xf, const Vector3 & a, const Vector3 & b, float r );
	void convex( quint32 shape, const Affine & xf, bool hasFaces, float radius );
	void compressedMesh( quint32 shape, const Affine & xf );

	const HkPackfile & pf;
	QVector<Vector3> & lines;
	Budget & budget;
	// shapes on the current recursion path, so a compound that (directly or indirectly) contains itself
	// is not followed again
	QVector<quint32> path;
};

void ShapeBuilder::sphere( const Affine & xf, const Vector3 & c, float r )
{
	const Vector3 ux( r, 0.0f, 0.0f ), uy( 0.0f, r, 0.0f ), uz( 0.0f, 0.0f, r );
	arc( xf, c, ux, uy, 0.0f, float( 2.0 * PI ), circleSegments );
	arc( xf, c, uy, uz, 0.0f, float( 2.0 * PI ), circleSegments );
	arc( xf, c, uz, ux, 0.0f, float( 2.0 * PI ), circleSegments );
}

void ShapeBuilder::capsule( const Affine & xf, const Vector3 & a, const Vector3 & b, float r )
{
	Vector3 d = b - a;
	float len = d.length();
	if ( !( len > 1.0e-6f ) ) {
		sphere( xf, a, r );
		return;
	}
	d = d / len;
	// two unit vectors perpendicular to the axis
	Vector3 u = Vector3::crossproduct( d, std::fabs( d[2] ) < 0.9f ? Vector3( 0.0f, 0.0f, 1.0f ) : Vector3( 1.0f, 0.0f, 0.0f ) );
	u.normalize();
	Vector3 v = Vector3::crossproduct( d, u );
	u = u * r;
	v = v * r;
	const Vector3 dr = d * r;

	arc( xf, a, u, v, 0.0f, float( 2.0 * PI ), circleSegments );
	arc( xf, b, u, v, 0.0f, float( 2.0 * PI ), circleSegments );
	line( xf, a + u, b + u );
	line( xf, a - u, b - u );
	line( xf, a + v, b + v );
	line( xf, a - v, b - v );
	// end caps: half circles from one side of the end ring, over the tip, to the other side
	// (angle 0 = +u, pi/2 = the tip away from the other end, pi = -u)
	const float pi = float( PI );
	arc( xf, a, u, -dr, 0.0f, pi, circleSegments / 2 );
	arc( xf, a, v, -dr, 0.0f, pi, circleSegments / 2 );
	arc( xf, b, u, dr, 0.0f, pi, circleSegments / 2 );
	arc( xf, b, v, dr, 0.0f, pi, circleSegments / 2 );
}

void ShapeBuilder::convex( quint32 shape, const Affine & xf, bool hasFaces, float radius )
{
	const HkSpan vertexSpan = pf.relArray( shape + convexVertices );
	const quint32 numVerts = vertexSpan.count;
	if ( !numVerts )
		return;
	QVector<Vector3> verts( static_cast<qsizetype>( numVerts ) );
	for ( quint32 i = 0; i < numVerts; i++ )
		verts[i] = pf.getVector3( vertexSpan.start + i * vertexSize );

	if ( hasFaces ) {
		const HkSpan planes = pf.relArray( shape + polytopePlanes );
		const HkSpan faces = pf.relArray( shape + polytopeFaces );
		const HkSpan indices = pf.relArray( shape + polytopeIndices );
		auto faceFirst = [&]( quint32 f ) -> quint32 { return pf.get<quint16>( faces.start + f * faceSize + faceFirstIndex ); };
		auto faceCount = [&]( quint32 f ) -> quint32 { return pf.get<quint8>( faces.start + f * faceSize + faceNumIndices ); };
		auto index = [&]( quint32 i ) -> quint32 { return pf.get<quint8>( indices.start + i ); };

		// Havok expands the shape by its convex radius: every face moves out along its normal.
		// Move each vertex to where its (moved) faces meet, least squares for more than three faces.
		if ( radius > 0.0f && planes.count >= faces.count ) {
			static const float zero[9] = {};
			const qsizetype nv = static_cast<qsizetype>( numVerts );
			QVector<Matrix> sumNN( nv, Matrix( zero ) );
			QVector<Vector3> sumN( nv );
			QVector<int> count( nv, 0 );
			for ( quint32 f = 0; f < faces.count; f++ ) {
				const Vector3 n = pf.getVector3( planes.start + f * planeSize );
				const quint32 first = faceFirst( f );
				const quint32 k = faceCount( f );
				for ( quint32 j = 0; j < k && first + j < indices.count; j++ ) {
					const quint32 v = index( first + j );
					if ( v >= numVerts )
						continue;
					for ( int r = 0; r < 3; r++ ) {
						for ( int c = 0; c < 3; c++ )
							sumNN[v]( r, c ) += n[r] * n[c];
					}
					sumN[v] += n;
					count[v]++;
				}
			}
			for ( quint32 v = 0; v < numVerts; v++ ) {
				if ( !count[v] )
					continue;
				const Matrix & m = sumNN[v];
				const float det = m( 0, 0 ) * ( m( 1, 1 ) * m( 2, 2 ) - m( 1, 2 ) * m( 2, 1 ) )
								- m( 0, 1 ) * ( m( 1, 0 ) * m( 2, 2 ) - m( 1, 2 ) * m( 2, 0 ) )
								+ m( 0, 2 ) * ( m( 1, 0 ) * m( 2, 1 ) - m( 1, 1 ) * m( 2, 0 ) );
				Vector3 offset;
				if ( std::fabs( det ) > 1.0e-4f ) {
					offset = m.inverted() * sumN[v];
					// very sharp corners would shoot out; limit the offset to 3 radii
					if ( offset.length() > 3.0f )
						offset = offset / offset.length() * 3.0f;
				} else {
					offset = sumN[v];
					offset.normalize();
				}
				verts[v] += offset * radius;
			}
		}

		for ( quint32 f = 0; f < faces.count; f++ ) {
			const quint32 first = faceFirst( f );
			const quint32 n = faceCount( f );
			if ( n < 2 || first + n > indices.count )
				continue;
			for ( quint32 k = 0; k < n; k++ ) {
				const quint32 i0 = index( first + k );
				const quint32 i1 = index( first + ( k + 1 ) % n );
				if ( i0 < numVerts && i1 < numVerts )
					line( xf, verts.at( i0 ), verts.at( i1 ) );
			}
		}
		if ( faces.count )
			return;
	}

	// no face data: mark each vertex with a small cross
	const float s = 0.005f;
	for ( const Vector3 & p : verts ) {
		line( xf, p - Vector3( s, 0.0f, 0.0f ), p + Vector3( s, 0.0f, 0.0f ) );
		line( xf, p - Vector3( 0.0f, s, 0.0f ), p + Vector3( 0.0f, s, 0.0f ) );
		line( xf, p - Vector3( 0.0f, 0.0f, s ), p + Vector3( 0.0f, 0.0f, s ) );
	}
}

void ShapeBuilder::compressedMesh( quint32 shape, const Affine & xf )
{
	quint32 data;
	if ( !pf.pointer( shape + compressedMeshData, data ) )
		return;
	const quint32 tree = data + meshTree;
	const Vector3 domainMin = pf.getVector3( tree + treeDomainMin );
	const Vector3 domainSize = pf.getVector3( tree + treeDomainMax ) - domainMin;

	const HkSpan sections = pf.array( tree + treeSections );
	const HkSpan primitives = pf.array( tree + treePrimitives );
	const HkSpan sharedIndices = pf.array( tree + treeSharedIndices );
	const HkSpan packedVerts = pf.array( tree + treePackedVertices );
	const HkSpan sharedVerts = pf.array( tree + treeSharedVertices );

	for ( quint32 s = 0; s < sections.count && s < 65536; s++ ) {
		const quint32 sec = sections.start + s * sectionSize;
		const Vector3 offset = pf.getVector3( sec + sectionCodecOffset );
		const Vector3 scale = pf.getVector3( sec + sectionCodecScale );
		const quint32 firstPacked = pf.get<quint32>( sec + sectionFirstPacked );
		const quint32 sharedRef = pf.get<quint32>( sec + sectionSharedVertices );	// count in the low 8 bits, offset above
		const quint32 primRef = pf.get<quint32>( sec + sectionPrimitives );
		const quint32 numPacked = pf.get<quint8>( sec + sectionNumPacked );
		// shared vertex indices are 16-bit; meshes with more than 65536 shared vertices (e.g. precombined
		// *_Physics.nif) select a block of 65536 per section with its page byte
		const quint32 page = pf.get<quint8>( sec + sectionPage );

		// section-local vertex index -> position: packed vertices first (11/11/10 bits, section codec),
		// then shared vertices (21/21/22 bits across the whole tree's domain)
		auto vertex = [&]( quint32 i, Vector3 & v ) -> bool {
			if ( i < numPacked ) {
				if ( firstPacked + i >= packedVerts.count )
					return false;
				const quint32 p = pf.get<quint32>( packedVerts.start + ( firstPacked + i ) * 4 );
				v = Vector3( offset[0] + scale[0] * float( p & 0x7FF ),
							 offset[1] + scale[1] * float( ( p >> 11 ) & 0x7FF ),
							 offset[2] + scale[2] * float( p >> 22 ) );
				return true;
			}
			const quint32 si = ( sharedRef >> 8 ) + ( i - numPacked );
			if ( si >= sharedIndices.count )
				return false;
			const quint32 sv = ( page << 16 ) + pf.get<quint16>( sharedIndices.start + si * 2 );
			if ( sv >= sharedVerts.count )
				return false;
			const quint64 p = pf.get<quint64>( sharedVerts.start + sv * 8 );
			v = Vector3( domainMin[0] + domainSize[0] * ( float( p & 0x1FFFFF ) / float( 0x1FFFFF ) ),
						 domainMin[1] + domainSize[1] * ( float( ( p >> 21 ) & 0x1FFFFF ) / float( 0x1FFFFF ) ),
						 domainMin[2] + domainSize[2] * ( float( p >> 42 ) / float( 0x3FFFFF ) ) );
			return true;
		};

		// primitives: a triangle if the last two indices are equal, otherwise a quad; drawn as outlines
		const quint32 firstPrim = primRef >> 8;
		const quint32 numPrims = primRef & 0xFF;
		for ( quint32 p = 0; p < numPrims; p++ ) {
			if ( firstPrim + p >= primitives.count )
				break;
			const quint32 prim = primitives.start + ( firstPrim + p ) * primitiveSize;
			const quint32 idx[4] = { pf.get<quint8>( prim ), pf.get<quint8>( prim + 1 ),
									 pf.get<quint8>( prim + 2 ), pf.get<quint8>( prim + 3 ) };
			const int n = ( idx[2] == idx[3] ) ? 3 : 4;
			// Skip degenerate primitives (repeated corners). Sections with flag 1 in precombined meshes hold
			// [a, b, b, b] entries that are not surfaces (likely another primitive type); drawn as triangles they
			// become long lines across the mesh.
			if ( idx[0] == idx[1] || idx[1] == idx[2] || idx[0] == idx[2] || ( n == 4 && ( idx[0] == idx[3] || idx[1] == idx[3] ) ) )
				continue;
			Vector3 v[4];
			bool ok = true;
			for ( int k = 0; k < n && ok; k++ )
				ok = vertex( idx[k], v[k] );
			if ( !ok )
				continue;
			for ( int k = 0; k < n; k++ )
				line( xf, v[k], v[( k + 1 ) % n] );
		}
	}
}

void ShapeBuilder::addShape( quint32 shape, const Affine & xf )
{
	// guard against malformed files: too deep, a shape that contains itself, or too much work overall
	if ( path.size() >= maxDepth || path.contains( shape ) || budget.shapeVisits <= 0 || budget.linePoints < 2 )
		return;
	budget.shapeVisits--;
	path.append( shape );
	addShapeContents( shape, xf );
	path.removeLast();
}

void ShapeBuilder::addShapeContents( quint32 shape, const Affine & xf )
{
	const QByteArray cls = pf.className( shape );
	const float convexRadius = pf.get<float>( shape + shapeConvexRadius );

	if ( cls == "hknpCapsuleShape" ) {
		capsule( xf, pf.getVector3( shape + capsulePointA ), pf.getVector3( shape + capsulePointB ), convexRadius );

	} else if ( cls == "hknpSphereShape" ) {
		const HkSpan verts = pf.relArray( shape + convexVertices );
		sphere( xf, verts.count ? pf.getVector3( verts.start ) : Vector3(), convexRadius );

	} else if ( cls == "hknpConvexPolytopeShape" ) {
		convex( shape, xf, true, convexRadius );

	} else if ( cls == "hknpConvexShape" ) {
		convex( shape, xf, false, convexRadius );

	} else if ( cls == "hknpCompressedMeshShape" ) {
		compressedMesh( shape, xf );

	} else if ( cls == "hknpScaledConvexShape" ) {
		quint32 core;
		if ( pf.pointer( shape + scaledCoreShape, core ) ) {
			const Vector3 s = pf.getVector3( shape + scaledScale );
			Affine m;
			m.x = Vector3( s[0], 0.0f, 0.0f );
			m.y = Vector3( 0.0f, s[1], 0.0f );
			m.z = Vector3( 0.0f, 0.0f, s[2] );
			m.t = pf.getVector3( shape + scaledTranslation );
			addShape( core, xf * m );
		}

	} else if ( cls == "hknpStaticCompoundShape" || cls == "hknpDynamicCompoundShape" ) {
		const HkSpan instances = pf.array( shape + compoundInstances );
		for ( quint32 i = 0; i < instances.count && i < 65536; i++ ) {
			const quint32 inst = instances.start + i * instanceSize;
			quint32 child;
			if ( !pf.pointer( inst + instanceShape, child ) )
				continue;	// free slot
			Affine m;
			m.x = pf.getVector3( inst + instanceRotation );
			m.y = pf.getVector3( inst + instanceRotation + 16 );
			m.z = pf.getVector3( inst + instanceRotation + 32 );
			m.t = pf.getVector3( inst + instanceTranslation );
			const Vector3 s = pf.getVector3( inst + instanceScale );
			if ( s[0] != 0.0f || s[1] != 0.0f || s[2] != 0.0f ) {
				m.x = m.x * s[0];
				m.y = m.y * s[1];
				m.z = m.z * s[2];
			}
			addShape( child, xf * m );
		}

	} else if ( !cls.isEmpty() ) {
		qDebug() << "FO4 collision: shape type not drawn yet:" << cls;
	}
}

//! One bhkPhysicsSystem's data: parsed once, wireframes built the first time a body is drawn
struct System
{
	QByteArray data;	// keeps the source alive: the packfile reader points into it, and its address is the cache key
	HkPackfile pf;
	quint32 root = quint32( -1 );	// hknpPhysicsSystemData, or -1 if the data cannot be read
	bool wireframesBuilt = false;
	QVector<HknpCollision::Body> bodies;
};

void buildWireframes( System & sys )
{
	sys.wireframesBuilt = true;
	if ( sys.root == quint32( -1 ) )
		return;
	// The shape is drawn in the space of the collision object's node (the body position and orientation are not
	// applied, see hknplayout.h).
	const HkSpan bodies = sys.pf.array( sys.root + systemBodies );
	Budget budget;	// shared by all bodies of this system
	for ( quint32 i = 0; i < bodies.count && i < 65536; i++ ) {
		const quint32 b = bodies.start + i * bodySize;
		HknpCollision::Body body;
		body.filterInfo = sys.pf.get<quint32>( b + bodyFilterInfo );
		quint32 shape;
		if ( sys.pf.pointer( b + bodyShape, shape ) )
			ShapeBuilder( sys.pf, body.lines, budget ).addShape( shape, Affine() );
		sys.bodies.append( body );
	}
}

// Parsed physics systems, keyed by the address of their "Binary Data". Each entry keeps a copy of that
// QByteArray, so the address stays unique while cached. Cleared when the scene is rebuilt.
QHash<const char *, System> & cache()
{
	static QHash<const char *, System> c;
	return c;
}

System * findSystem( const QByteArray & data )
{
	if ( data.isEmpty() )
		return nullptr;
	auto i = cache().find( data.constData() );
	if ( i == cache().end() ) {
		i = cache().insert( data.constData(), System() );
		i->data = data;		// the parser points into this copy, which shares the model's buffer
		if ( i->pf.parse( i->data ) )
			i->root = i->pf.findObject( "hknpPhysicsSystemData" );
	}
	return &i.value();
}

}	// namespace

const HkPackfile * HknpCollision::Detail::parsedSystem( const QByteArray & data, quint32 & root )
{
	const System * sys = findSystem( data );
	if ( !sys || sys->root == quint32( -1 ) )
		return nullptr;
	root = sys->root;
	return &sys->pf;
}

void HknpCollision::clearCache()
{
	cache().clear();
	Detail::clearViewRecords();
}

const HknpCollision::Body * HknpCollision::getBody( const QByteArray & data, quint32 bodyID )
{
	System * sys = findSystem( data );
	if ( !sys )
		return nullptr;
	if ( !sys->wireframesBuilt )
		buildWireframes( *sys );
	if ( bodyID >= quint32( sys->bodies.size() ) )
		return nullptr;
	return &( sys->bodies.at( bodyID ) );
}

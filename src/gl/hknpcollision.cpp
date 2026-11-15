#include "hknpcollision.h"

#include <QDebug>
#include <QHash>

#include <cmath>
#include <cstring>

//! @file hknpcollision.cpp Reads the Havok 2014 binary packfile embedded in bhkPhysicsSystem

/*
 * Fallout 4 stores collision as a complete Havok packfile ("hk_2014.1.0-r1", 64-bit, little-endian)
 * inside bhkPhysicsSystem. Its __types__ section is empty, so the class layouts below are built in.
 * Only the shape types needed for drawing are decoded; everything is read with bounds checks and
 * anything unknown is skipped.
 */

namespace
{

class Packfile
{
public:
	bool parse( const QByteArray & file );

	template <typename T> T get( quint32 offset ) const
	{
		T v{};
		if ( quint64( offset ) + sizeof( T ) <= quint64( size ) )
			std::memcpy( &v, data + offset, sizeof( T ) );
		return v;
	}
	Vector3 getVector3( quint32 offset ) const
	{
		return Vector3( get<float>( offset ), get<float>( offset + 4 ), get<float>( offset + 8 ) );
	}
	//! Follows the pointer stored at @p offset; returns false for a null pointer
	bool pointer( quint32 offset, quint32 & target ) const
	{
		auto i = pointers.constFind( offset );
		if ( i == pointers.constEnd() )
			return false;
		target = i.value();
		return true;
	}
	//! Resolves an hkRelArray (16-bit count, 16-bit offset from the hkRelArray itself)
	quint32 relArray( quint32 offset, quint32 & start ) const
	{
		start = offset + get<quint16>( offset + 2 );
		return get<quint16>( offset );
	}
	//! Resolves an hkArray (pointer, 32-bit count, capacity and flags)
	quint32 array( quint32 offset, quint32 & start ) const
	{
		if ( !pointer( offset, start ) )
			return 0;
		return quint32( std::max< qint32 >( get<qint32>( offset + 8 ), 0 ) );
	}
	QByteArray className( quint32 offset ) const { return classes.value( offset ); }
	quint32 findObject( const char * name ) const
	{
		for ( auto i = classes.constBegin(); i != classes.constEnd(); i++ ) {
			if ( i.value() == name )
				return i.key();
		}
		return quint32( -1 );
	}

private:
	const char * data = nullptr;	// start of the __data__ section
	quint32 size = 0;				// size of the object data (up to the local fixups)
	QHash<quint32, quint32> pointers;
	QHash<quint32, QByteArray> classes;	// object offset -> class name
};

bool Packfile::parse( const QByteArray & file )
{
	static const unsigned char magic[8] = { 0x57, 0xE0, 0xE0, 0x57, 0x10, 0xC0, 0xC0, 0x10 };

	const char * f = file.constData();
	const quint32 fileSize = quint32( file.size() );
	auto u32 = [&]( quint32 o ) -> quint32 {
		quint32 v = 0;
		if ( quint64( o ) + 4 <= fileSize )
			std::memcpy( &v, f + o, 4 );
		return v;
	};

	if ( fileSize < 0x40 || std::memcmp( f, magic, 8 ) != 0 )
		return false;
	// layout rules: 8-byte pointers, little-endian
	if ( f[0x10] != 8 || f[0x11] != 1 )
		return false;

	const quint32 version = u32( 0x0C );
	const qint32 numSections = qint32( u32( 0x14 ) );
	qint32 predicatePadding = 0;
	if ( version >= 11 )
		predicatePadding = std::max< qint16 >( qint16( u32( 0x3C ) >> 16 ), 0 );

	quint32 classStart = 0, classEnd = 0;
	quint32 dataStart = 0, localFixups = 0, globalFixups = 0, virtualFixups = 0, exports = 0;
	int dataSection = -1;
	for ( qint32 i = 0; i < numSections && i < 16; i++ ) {
		const quint32 h = 0x40 + quint32( predicatePadding ) + quint32( i ) * 0x40;
		if ( h + 0x40 > fileSize )
			return false;
		const QByteArray name( f + h, qstrnlen( f + h, 19 ) );
		const quint32 start = u32( h + 20 );
		if ( name == "__classnames__" ) {
			classStart = start;
			classEnd = start + u32( h + 24 );
		} else if ( name == "__data__" ) {
			dataSection = int( i );
			dataStart = start;
			localFixups = u32( h + 24 );
			globalFixups = u32( h + 28 );
			virtualFixups = u32( h + 32 );
			exports = u32( h + 36 );
		}
	}
	if ( dataSection < 0 || !classStart || classEnd > fileSize || quint64( dataStart ) + exports > fileSize
		|| localFixups > globalFixups || globalFixups > virtualFixups || virtualFixups > exports ) {
		return false;
	}

	data = f + dataStart;
	size = localFixups;

	// local fixups: (source, target) pairs within the data section; -1 marks padding
	for ( quint32 o = localFixups; o + 8 <= globalFixups; o += 8 ) {
		qint32 src = qint32( u32( dataStart + o ) );
		if ( src >= 0 )
			pointers.insert( quint32( src ), u32( dataStart + o + 4 ) );
	}
	// global fixups: (source, target section, target); only pointers into the data section are used
	for ( quint32 o = globalFixups; o + 12 <= virtualFixups; o += 12 ) {
		qint32 src = qint32( u32( dataStart + o ) );
		if ( src >= 0 && qint32( u32( dataStart + o + 4 ) ) == dataSection )
			pointers.insert( quint32( src ), u32( dataStart + o + 8 ) );
	}
	// virtual fixups: (object, class name section, offset of the name in that section)
	for ( quint32 o = virtualFixups; o + 12 <= exports; o += 12 ) {
		qint32 obj = qint32( u32( dataStart + o ) );
		if ( obj < 0 )
			continue;
		quint32 nameOffset = classStart + u32( dataStart + o + 8 );
		if ( nameOffset >= classEnd )
			continue;
		classes.insert( quint32( obj ), QByteArray( f + nameOffset, qstrnlen( f + nameOffset, classEnd - nameOffset ) ) );
	}
	return !classes.isEmpty();
}

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

class ShapeBuilder
{
public:
	ShapeBuilder( const Packfile & p, QVector<Vector3> & l ) : pf( p ), lines( l ) {}

	void addShape( quint32 shape, const Affine & xf, int depth = 0 );

private:
	static constexpr int circleSegments = 24;

	void line( const Affine & xf, const Vector3 & a, const Vector3 & b )
	{
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

	const Packfile & pf;
	QVector<Vector3> & lines;
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
	const float halfPi = float( PI * 0.5 );

	arc( xf, a, u, v, 0.0f, float( 2.0 * PI ), circleSegments );
	arc( xf, b, u, v, 0.0f, float( 2.0 * PI ), circleSegments );
	line( xf, a + u, b + u );
	line( xf, a - u, b - u );
	line( xf, a + v, b + v );
	line( xf, a - v, b - v );
	// end caps: half circles bulging away from the other end
	arc( xf, a, u, -dr, -halfPi, halfPi, circleSegments / 2 );
	arc( xf, a, v, -dr, -halfPi, halfPi, circleSegments / 2 );
	arc( xf, b, u, dr, -halfPi, halfPi, circleSegments / 2 );
	arc( xf, b, v, dr, -halfPi, halfPi, circleSegments / 2 );
}

void ShapeBuilder::convex( quint32 shape, const Affine & xf, bool hasFaces, float radius )
{
	// hknpConvexShape::vertices at 48; the w component holds the vertex index, not a coordinate
	quint32 vertStart;
	const quint32 numVerts = pf.relArray( shape + 48, vertStart );
	if ( !numVerts )
		return;
	QVector<Vector3> verts( static_cast<qsizetype>( numVerts ) );
	for ( quint32 i = 0; i < numVerts; i++ )
		verts[i] = pf.getVector3( vertStart + i * 16 );
	auto vertex = [&]( quint32 i ) { return verts.at( i ); };

	if ( hasFaces ) {
		// hknpConvexPolytopeShape: planes (normal, distance) at 64, one per face,
		// faces at 68 (firstIndex u16, numIndices u8, minHalfAngle u8), indices (u8) at 72
		quint32 planeStart, faceStart, indexStart;
		const quint32 numPlanes = pf.relArray( shape + 64, planeStart );
		const quint32 numFaces = pf.relArray( shape + 68, faceStart );
		const quint32 numIndices = pf.relArray( shape + 72, indexStart );

		// Havok expands the shape by its convex radius: every face moves out along its normal.
		// Move each vertex to where its (moved) faces meet, least squares for more than three faces.
		if ( radius > 0.0f && numPlanes >= numFaces ) {
			static const float zero[9] = {};
			const qsizetype nv = static_cast<qsizetype>( numVerts );
			QVector<Matrix> sumNN( nv, Matrix( zero ) );
			QVector<Vector3> sumN( nv );
			QVector<int> count( nv, 0 );
			for ( quint32 f = 0; f < numFaces; f++ ) {
				const Vector3 n = pf.getVector3( planeStart + f * 16 );
				const quint32 first = pf.get<quint16>( faceStart + f * 4 );
				const quint32 k = pf.get<quint8>( faceStart + f * 4 + 2 );
				for ( quint32 j = 0; j < k && first + j < numIndices; j++ ) {
					const quint32 v = pf.get<quint8>( indexStart + first + j );
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

		for ( quint32 f = 0; f < numFaces; f++ ) {
			const quint32 first = pf.get<quint16>( faceStart + f * 4 );
			const quint32 n = pf.get<quint8>( faceStart + f * 4 + 2 );
			if ( n < 2 || first + n > numIndices )
				continue;
			for ( quint32 k = 0; k < n; k++ ) {
				const quint32 i0 = pf.get<quint8>( indexStart + first + k );
				const quint32 i1 = pf.get<quint8>( indexStart + first + ( k + 1 ) % n );
				if ( i0 < numVerts && i1 < numVerts )
					line( xf, vertex( i0 ), vertex( i1 ) );
			}
		}
		if ( numFaces )
			return;
	}

	// no face data: mark each vertex with a small cross
	const float s = 0.005f;
	for ( quint32 i = 0; i < numVerts; i++ ) {
		const Vector3 p = vertex( i );
		line( xf, p - Vector3( s, 0.0f, 0.0f ), p + Vector3( s, 0.0f, 0.0f ) );
		line( xf, p - Vector3( 0.0f, s, 0.0f ), p + Vector3( 0.0f, s, 0.0f ) );
		line( xf, p - Vector3( 0.0f, 0.0f, s ), p + Vector3( 0.0f, 0.0f, s ) );
	}
}

void ShapeBuilder::compressedMesh( quint32 shape, const Affine & xf )
{
	// hknpCompressedMeshShape::data at 96 -> hknpCompressedMeshShapeData, whose meshTree
	// (hknpCompressedMeshShapeTree, an hkcdStaticMeshTree) starts at 16:
	//   domain AABB min/max at 16/32, sections at 64 (96 bytes each), primitives at 80 (4 u8 indices each),
	//   sharedVerticesIndex (u16) at 96, packedVertices (u32) at 112, sharedVertices (u64) at 128
	quint32 data;
	if ( !pf.pointer( shape + 96, data ) )
		return;
	const quint32 tree = data + 16;
	const Vector3 domainMin = pf.getVector3( tree + 16 );
	const Vector3 domainSize = pf.getVector3( tree + 32 ) - domainMin;

	quint32 sections = 0, primitives = 0, sharedIndices = 0, packedVerts = 0, sharedVerts = 0;
	const quint32 numSections = pf.array( tree + 64, sections );
	const quint32 numPrimitives = pf.array( tree + 80, primitives );
	const quint32 numSharedIndices = pf.array( tree + 96, sharedIndices );
	const quint32 numPackedVerts = pf.array( tree + 112, packedVerts );
	const quint32 numSharedVerts = pf.array( tree + 128, sharedVerts );

	for ( quint32 s = 0; s < numSections && s < 65536; s++ ) {
		const quint32 sec = sections + s * 96;
		// codecParms: offset xyz at 48, scale xyz at 60
		const Vector3 offset = pf.getVector3( sec + 48 );
		const Vector3 scale = pf.getVector3( sec + 60 );
		const quint32 firstPacked = pf.get<quint32>( sec + 72 );
		const quint32 sharedRef = pf.get<quint32>( sec + 76 );	// count in the low 8 bits, offset above
		const quint32 primRef = pf.get<quint32>( sec + 80 );
		const quint32 numPacked = pf.get<quint8>( sec + 88 );

		// section-local vertex index -> position: packed vertices first (11/11/10 bits, section codec),
		// then shared vertices (21/21/22 bits across the whole tree's domain)
		auto vertex = [&]( quint32 i, Vector3 & v ) -> bool {
			if ( i < numPacked ) {
				if ( firstPacked + i >= numPackedVerts )
					return false;
				const quint32 p = pf.get<quint32>( packedVerts + ( firstPacked + i ) * 4 );
				v = Vector3( offset[0] + scale[0] * float( p & 0x7FF ),
							 offset[1] + scale[1] * float( ( p >> 11 ) & 0x7FF ),
							 offset[2] + scale[2] * float( p >> 22 ) );
				return true;
			}
			const quint32 si = ( sharedRef >> 8 ) + ( i - numPacked );
			if ( si >= numSharedIndices )
				return false;
			const quint32 sv = pf.get<quint16>( sharedIndices + si * 2 );
			if ( sv >= numSharedVerts )
				return false;
			const quint64 p = pf.get<quint64>( sharedVerts + sv * 8 );
			v = Vector3( domainMin[0] + domainSize[0] * ( float( p & 0x1FFFFF ) / float( 0x1FFFFF ) ),
						 domainMin[1] + domainSize[1] * ( float( ( p >> 21 ) & 0x1FFFFF ) / float( 0x1FFFFF ) ),
						 domainMin[2] + domainSize[2] * ( float( p >> 42 ) / float( 0x3FFFFF ) ) );
			return true;
		};

		// primitives: a triangle if the last two indices are equal, otherwise a quad; drawn as outlines
		const quint32 firstPrim = primRef >> 8;
		const quint32 numPrims = primRef & 0xFF;
		for ( quint32 p = 0; p < numPrims; p++ ) {
			if ( firstPrim + p >= numPrimitives )
				break;
			const quint32 prim = primitives + ( firstPrim + p ) * 4;
			const quint32 idx[4] = { pf.get<quint8>( prim ), pf.get<quint8>( prim + 1 ),
									 pf.get<quint8>( prim + 2 ), pf.get<quint8>( prim + 3 ) };
			const int n = ( idx[2] == idx[3] ) ? 3 : 4;
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

void ShapeBuilder::addShape( quint32 shape, const Affine & xf, int depth )
{
	if ( depth > 16 )
		return;

	const QByteArray cls = pf.className( shape );
	// hknpShape::convexRadius
	const float convexRadius = pf.get<float>( shape + 20 );

	if ( cls == "hknpCapsuleShape" ) {
		capsule( xf, pf.getVector3( shape + 80 ), pf.getVector3( shape + 96 ), convexRadius );

	} else if ( cls == "hknpSphereShape" ) {
		quint32 vertStart;
		Vector3 c;
		if ( pf.relArray( shape + 48, vertStart ) )
			c = pf.getVector3( vertStart );
		sphere( xf, c, convexRadius );

	} else if ( cls == "hknpConvexPolytopeShape" ) {
		convex( shape, xf, true, convexRadius );

	} else if ( cls == "hknpConvexShape" ) {
		convex( shape, xf, false, convexRadius );

	} else if ( cls == "hknpCompressedMeshShape" ) {
		compressedMesh( shape, xf );

	} else if ( cls == "hknpScaledConvexShape" ) {
		// hknpDecoratorShape::coreShape at 48, hknpScaledConvexShapeBase::scale at 64, translation at 80
		quint32 core;
		if ( pf.pointer( shape + 48, core ) ) {
			const Vector3 s = pf.getVector3( shape + 64 );
			Affine m;
			m.x = Vector3( s[0], 0.0f, 0.0f );
			m.y = Vector3( 0.0f, s[1], 0.0f );
			m.z = Vector3( 0.0f, 0.0f, s[2] );
			m.t = pf.getVector3( shape + 80 );
			addShape( core, xf * m, depth + 1 );
		}

	} else if ( cls == "hknpStaticCompoundShape" || cls == "hknpDynamicCompoundShape" ) {
		// hknpCompoundShape::instances (hkFreeListArray, elements hkArray first) at 96
		// hknpShapeInstance (128 bytes): hkTransform at 0 (3 rotation columns, translation), scale at 64, shape at 80
		quint32 start;
		const quint32 n = pf.array( shape + 96, start );
		for ( quint32 i = 0; i < n && i < 65536; i++ ) {
			const quint32 inst = start + i * 128;
			quint32 child;
			if ( !pf.pointer( inst + 80, child ) )
				continue;	// free slot
			Affine m;
			m.x = pf.getVector3( inst );
			m.y = pf.getVector3( inst + 16 );
			m.z = pf.getVector3( inst + 32 );
			m.t = pf.getVector3( inst + 48 );
			const Vector3 s = pf.getVector3( inst + 64 );
			if ( s[0] != 0.0f || s[1] != 0.0f || s[2] != 0.0f ) {
				m.x = m.x * s[0];
				m.y = m.y * s[1];
				m.z = m.z * s[2];
			}
			addShape( child, xf * m, depth + 1 );
		}

	} else if ( !cls.isEmpty() ) {
		qDebug() << "FO4 collision: shape type not drawn yet:" << cls;
	}
}

struct System
{
	QByteArray data;	// keeps the source alive, so the cache key (its address) stays unique
	QVector<HknpCollision::Body> bodies;
};

void loadSystem( System & sys )
{
	// bhkPhysicsSystem "Binary Data" is the packfile itself
	Packfile pf;
	if ( !pf.parse( sys.data ) )
		return;
	const quint32 root = pf.findObject( "hknpPhysicsSystemData" );
	if ( root == quint32( -1 ) )
		return;

	// hknpPhysicsSystemData::bodyCinfos at 64; hknpBodyCinfo is 96 bytes:
	// shape pointer at 0, collisionFilterInfo at 20, position at 48, orientation at 64.
	// The shape is drawn in the space of the collision object's node. The body position and orientation
	// only record where the body was at export (e.g. the node's own translation, or the centre of a box
	// whose vertices are already in node space), so applying them would place the shape twice.
	quint32 start;
	const quint32 n = pf.array( root + 64, start );
	for ( quint32 i = 0; i < n && i < 65536; i++ ) {
		const quint32 b = start + i * 96;
		HknpCollision::Body body;
		body.filterInfo = pf.get<quint32>( b + 20 );
		quint32 shape;
		if ( pf.pointer( b, shape ) )
			ShapeBuilder( pf, body.lines ).addShape( shape, Affine() );
		sys.bodies.append( body );
	}
}

}	// namespace

const HknpCollision::Body * HknpCollision::getBody( const QByteArray & data, quint32 bodyID )
{
	static QHash<const char *, System> cache;

	if ( data.isEmpty() )
		return nullptr;

	auto i = cache.find( data.constData() );
	if ( i == cache.end() ) {
		if ( cache.size() >= 256 )
			cache.clear();
		System sys;
		sys.data = data;
		loadSystem( sys );
		i = cache.insert( data.constData(), sys );
	}
	if ( bodyID >= quint32( i->bodies.size() ) )
		return nullptr;
	return &( i->bodies.at( bodyID ) );
}

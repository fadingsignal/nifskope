#include "hknpcollision.h"

#include "model/nifmodel.h"

#include <QDebug>
#include <QHash>
#include <QPair>

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
	//! How many elements of @p stride bytes starting at @p start fit in the data (limits counts read from the file)
	quint32 available( quint32 start, quint32 stride ) const
	{
		return ( start < size && stride ) ? ( size - start ) / stride : 0;
	}
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

//! Work limits for one physics system, shared by all of its bodies, so a malformed file cannot freeze the viewer.
//! The largest vanilla collision (precombined *_Physics.nif compressed meshes, ~600k triangles) needs about
//! 4.5M line points, far below these limits.
struct Budget
{
	qsizetype linePoints = 32 * 1024 * 1024;
	qsizetype shapeVisits = 1024 * 1024;
};

class ShapeBuilder
{
public:
	ShapeBuilder( const Packfile & p, QVector<Vector3> & l, Budget & b ) : pf( p ), lines( l ), budget( b ) {}

	void addShape( quint32 shape, const Affine & xf );

private:
	void addShapeContents( quint32 shape, const Affine & xf );
	static constexpr int circleSegments = 24;
	static constexpr int maxDepth = 16;

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

	const Packfile & pf;
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
		// shared vertex indices are 16-bit; meshes with more than 65536 shared vertices (e.g. precombined
		// *_Physics.nif) select a block of 65536 per section with its page byte
		const quint32 page = pf.get<quint8>( sec + 92 );

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
			const quint32 sv = ( page << 16 ) + pf.get<quint16>( sharedIndices + si * 2 );
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
			addShape( core, xf * m );
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
			addShape( child, xf * m );
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
	quint32 start = 0;
	const quint32 n = pf.array( root + 64, start );
	Budget budget;	// shared by all bodies of this system
	for ( quint32 i = 0; i < n && i < 65536; i++ ) {
		const quint32 b = start + i * 96;
		HknpCollision::Body body;
		body.filterInfo = pf.get<quint32>( b + 20 );
		quint32 shape;
		if ( pf.pointer( b, shape ) )
			ShapeBuilder( pf, body.lines, budget ).addShape( shape, Affine() );
		sys.bodies.append( body );
	}
}

/*
 * Read-only views for the block tree: fills the abstract nif.xml structs hknpViewPhysicsSystem and hknpViewBody
 * (see build/nif.xml) from the packfile. Values are in Havok units, as stored.
 */
class ViewFiller
{
public:
	ViewFiller( NifModel * n, const Packfile & p ) : nif( n ), pf( p ) {}

	void system( NifItem * item, quint32 root );
	void body( NifItem * item, quint32 root, quint32 bodyID );

private:
	static constexpr int maxDepth = 8;
	static constexpr quint32 maxArray = 65536;

	NifItem * child( NifItem * parent, const char * name ) const
	{
		return parent ? nif->getItem( parent, QLatin1StringView( name ) ) : nullptr;
	}
	template <typename T> void put( NifItem * parent, const char * name, const T & v ) const
	{
		NifItem::set<T>( child( parent, name ), v );
	}
	// set an array's count, re-evaluate the conditions of the struct and resize the array;
	// skips arrays that the struct does not have, or that are switched off by their condition
	// (the model does not return those, and asking for them by name reports an error)
	void resize( NifItem * parent, const char * count, const char * array, quint32 n ) const
	{
		if ( !parent || !child( parent, array ) )
			return;
		put<quint32>( parent, count, n );
		for ( auto c : parent->children() )
			c->invalidateCondition();
		nif->updateArraySize( parent, array );
	}
	static float half( quint16 h )
	{
		const quint32 bits = quint32( h ) << 16;
		float f;
		std::memcpy( &f, &bits, 4 );
		return f;
	}
	// hkPackedVector3: three int16 values and a scale (the high half of a float) in the fourth
	Vector3 packedVector( quint32 offset ) const
	{
		const quint32 bits = quint32( pf.get<quint16>( offset + 6 ) ) << 16;
		float scale;
		std::memcpy( &scale, &bits, 4 );
		scale *= 65536.0f;
		return Vector3( pf.get<qint16>( offset ) * scale, pf.get<qint16>( offset + 2 ) * scale,
						pf.get<qint16>( offset + 4 ) * scale );
	}

	/*! Number of array elements to show: the count from the file, limited to what the data holds and to the
	 *  remaining row budget (each element costs @p rowsEach rows). A malformed file can repeat a large shape in
	 *  many bodies or instances; the budget keeps the whole view bounded. Real files stay far below it.
	 */
	quint32 take( quint32 n, quint32 start, quint32 stride, int rowsEach )
	{
		n = std::min( { n, maxArray, pf.available( start, stride ) } );
		const qsizetype affordable = std::max< qsizetype >( rowBudget, 0 ) / rowsEach;
		n = quint32( std::min< qsizetype >( n, affordable ) );
		rowBudget -= qsizetype( n ) * rowsEach;
		return n;
	}

	quint32 shapeType( quint32 shape ) const;
	void shape( NifItem * item, quint32 shape, int depth );
	void massProperties( NifItem * item, quint32 shape ) const;

	NifModel * nif;
	const Packfile & pf;
	qsizetype rowBudget = 500000;	// rows left for this fill
};

quint32 ViewFiller::shapeType( quint32 shape ) const
{
	// values of the nif.xml enum hknpViewShapeType
	static const char * const names[] = {
		nullptr, "hknpConvexPolytopeShape", "hknpConvexShape", "hknpCapsuleShape", "hknpSphereShape",
		"hknpScaledConvexShape", "hknpStaticCompoundShape", "hknpDynamicCompoundShape", "hknpCompressedMeshShape"
	};
	const QByteArray cls = pf.className( shape );
	for ( quint32 i = 1; i < sizeof( names ) / sizeof( names[0] ); i++ ) {
		if ( cls == names[i] )
			return i;
	}
	return 0;
}

void ViewFiller::massProperties( NifItem * item, quint32 shape ) const
{
	// hknpShape::properties at 32 -> hkRefCountedProperties: entries (object*, key u16) at 0, 16 bytes each;
	// key 0xF100 -> hknpShapeMassProperties, hkCompressedMassProperties at 16:
	// centerOfMass (packed) 0, inertia (packed) 8, majorAxisSpace 16, mass 24, volume 28
	quint32 props = 0, entries = 0, massProps = 0;
	bool found = false;
	if ( pf.pointer( shape + 32, props ) ) {
		const quint32 n = pf.array( props, entries );
		for ( quint32 i = 0; i < n && i < 64 && !found; i++ ) {
			if ( pf.get<quint16>( entries + i * 16 + 8 ) == 0xF100 && pf.pointer( entries + i * 16, massProps ) )
				found = true;
		}
	}
	put<bool>( item, "Has Mass Properties", found );
	for ( auto c : item->children() )
		c->invalidateCondition();
	if ( !found )
		return;
	NifItem * m = child( item, "Mass Properties" );
	const quint32 c = massProps + 16;
	put<float>( m, "Volume", pf.get<float>( c + 28 ) );
	put<float>( m, "Mass", pf.get<float>( c + 24 ) );
	put<Vector3>( m, "Center Of Mass", packedVector( c ) );
	put<Vector3>( m, "Inertia", packedVector( c + 8 ) );
}

void ViewFiller::shape( NifItem * item, quint32 shape, int depth )
{
	if ( !item )
		return;
	rowBudget -= 25;	// the shape's own fields
	const quint32 type = ( depth <= maxDepth ? shapeType( shape ) : 0 );
	put<quint32>( item, "Shape Type", type );
	put<quint32>( item, "UserData", quint32( pf.get<quint64>( shape + 24 ) ) );
	put<float>( item, "Convex Radius", pf.get<float>( shape + 20 ) );
	for ( auto c : item->children() )
		c->invalidateCondition();

	// zero the counts, so arrays that are hidden for this shape type stay empty if they are shown later
	put<quint32>( item, "Vert Count", 0 );
	put<quint32>( item, "Plane Count", 0 );
	put<quint32>( item, "Face Count", 0 );
	put<quint32>( item, "Num Instances", 0 );

	switch ( type ) {
	case 1:		// convex polytope
	case 2: {	// convex: vertices at 48 (hkRelArray of hkVector4)
		quint32 start = 0;
		const quint32 nvRaw = pf.relArray( shape + 48, start );	// read the count first: it also sets start
		const quint32 nv = take( nvRaw, start, 16, 1 );
		resize( item, "Vert Count", "Vertices", nv );
		if ( NifItem * a = child( item, "Vertices" ) ) {
			for ( quint32 i = 0; i < nv && int( i ) < a->childCount(); i++ )
				NifItem::set<Vector3>( a->child( int( i ) ), pf.getVector3( start + i * 16 ) );
		}
		if ( type == 1 ) {
			// planes at 64 (hkVector4), faces at 68 (firstIndex u16, numIndices u8, minHalfAngle u8)
			const quint32 npRaw = pf.relArray( shape + 64, start );	// read the count first: it also sets start
			const quint32 np = take( npRaw, start, 16, 1 );
			resize( item, "Plane Count", "Planes", np );
			if ( NifItem * a = child( item, "Planes" ) ) {
				for ( quint32 i = 0; i < np && int( i ) < a->childCount(); i++ ) {
					const quint32 o = start + i * 16;
					NifItem::set<Vector4>( a->child( int( i ) ),
										   Vector4( pf.get<float>( o ), pf.get<float>( o + 4 ),
													pf.get<float>( o + 8 ), pf.get<float>( o + 12 ) ) );
				}
			}
			const quint32 nfRaw = pf.relArray( shape + 68, start );	// read the count first: it also sets start
			const quint32 nf = take( nfRaw, start, 4, 3 );
			resize( item, "Face Count", "Faces", nf );
			if ( NifItem * a = child( item, "Faces" ) ) {
				for ( quint32 i = 0; i < nf && int( i ) < a->childCount(); i++ ) {
					NifItem * f = a->child( int( i ) );
					put<quint16>( f, "First Index", pf.get<quint16>( start + i * 4 ) );
					put<quint8>( f, "Num Indices", pf.get<quint8>( start + i * 4 + 2 ) );
				}
			}
		}
		break;
	}
	case 3:		// capsule: a at 80, b at 96, radius = convex radius
		put<Vector3>( item, "Point A", pf.getVector3( shape + 80 ) );
		put<Vector3>( item, "Point B", pf.getVector3( shape + 96 ) );
		break;
	case 4: {	// sphere: centre = vertices[0], radius = convex radius
		quint32 start = 0;
		if ( pf.relArray( shape + 48, start ) )
			put<Vector3>( item, "Center", pf.getVector3( start ) );
		break;
	}
	case 5: {	// scaled convex: core shape at 48, scale at 64, translation at 80
		put<Vector3>( item, "Scale", pf.getVector3( shape + 64 ) );
		put<Vector3>( item, "Translation", pf.getVector3( shape + 80 ) );
		quint32 core;
		if ( NifItem * c = child( item, "Core Shape" ); c && pf.pointer( shape + 48, core ) )
			this->shape( c, core, depth + 1 );
		break;
	}
	case 6:
	case 7: {	// compound: instances (128 bytes each) at 96: rotation columns 0/16/32, translation 48,
				// scale 64, shape 80, shapeTag 88
		if ( !child( item, "Instances" ) )
			break;
		quint32 start = 0;
		// each instance creates its fields and a nested child shape struct (~30 rows)
		const quint32 nRaw = pf.array( shape + 96, start );	// read the count first: it also sets start
		const quint32 n = take( nRaw, start, 128, 30 );
		resize( item, "Num Instances", "Instances", n );
		NifItem * a = child( item, "Instances" );
		for ( quint32 i = 0; a && i < n && int( i ) < a->childCount(); i++ ) {
			NifItem * inst = a->child( int( i ) );
			const quint32 o = start + i * 128;
			const Vector3 cx = pf.getVector3( o ), cy = pf.getVector3( o + 16 ), cz = pf.getVector3( o + 32 );
			const float rot[9] = { cx[0], cy[0], cz[0], cx[1], cy[1], cz[1], cx[2], cy[2], cz[2] };
			put<quint16>( inst, "Instance ShapeTag", pf.get<quint16>( o + 88 ) );
			put<Vector3>( inst, "Instance Scale", pf.getVector3( o + 64 ) );
			put<Vector3>( inst, "Instance Translate", pf.getVector3( o + 48 ) );
			put<Matrix>( inst, "Instance Rotate", Matrix( rot ) );
			quint32 childShape;
			if ( pf.pointer( o + 80, childShape ) )
				this->shape( child( inst, "Shape" ), childShape, depth + 1 );
		}
		break;
	}
	case 8: {	// compressed mesh: data at 96, mesh tree at data + 16 (domain 16/32, sections 64, primitives 80)
		quint32 data;
		if ( child( item, "Num Sections" ) && pf.pointer( shape + 96, data ) ) {
			quint32 unused;
			put<quint32>( item, "Num Sections", pf.array( data + 16 + 64, unused ) );
			put<quint32>( item, "Num Primitives", pf.array( data + 16 + 80, unused ) );
			put<Vector3>( item, "Bounds Min", pf.getVector3( data + 16 + 16 ) );
			put<Vector3>( item, "Bounds Max", pf.getVector3( data + 16 + 32 ) );
		}
		break;
	}
	default:
		break;
	}
	massProperties( item, shape );
}

void ViewFiller::body( NifItem * item, quint32 root, quint32 bodyID )
{
	if ( !item )
		return;
	rowBudget -= 12;	// the body's own fields
	// hknpBodyCinfo (96 bytes): shape* 0, motionId 12, qualityId 16 (u8), materialId 18 (u16),
	// collisionFilterInfo 20, flags 24
	quint32 bodies = 0, motions = 0;
	// read each count first (it also sets the start offset), then limit it to the data
	const quint32 nBodiesRaw = pf.array( root + 64, bodies );
	const quint32 nBodies = std::min( nBodiesRaw, pf.available( bodies, 96 ) );
	const quint32 nMotionsRaw = pf.array( root + 48, motions );
	const quint32 nMotions = std::min( nMotionsRaw, pf.available( motions, 112 ) );
	if ( bodyID >= nBodies )
		return;
	const quint32 b = bodies + bodyID * 96;
	const quint32 filter = pf.get<quint32>( b + 20 );
	const quint32 motionID = pf.get<quint32>( b + 12 );
	put<quint32>( item, "Body ID", bodyID );
	put<quint32>( item, "Collision Filter", filter );
	put<quint32>( item, "Layer", filter & 0x7F );
	put<quint32>( item, "Flags", pf.get<quint32>( b + 24 ) );
	put<quint32>( item, "Motion ID", motionID );
	put<quint32>( item, "Material ID", pf.get<quint16>( b + 18 ) );
	put<quint32>( item, "Quality ID", pf.get<quint8>( b + 16 ) );
	float mass = 0.0f;
	if ( motionID < nMotions ) {
		const float invMass = pf.get<float>( motions + motionID * 112 + 4 );
		if ( invMass > 0.0f )
			mass = 1.0f / invMass;
	}
	put<float>( item, "Mass", mass );
	quint32 s;
	if ( pf.pointer( b, s ) )
		shape( child( item, "Shape" ), s, 0 );
}

void ViewFiller::system( NifItem * item, quint32 root )
{
	if ( !item )
		return;
	quint32 start = 0;

	// each body creates a body struct with a nested shape struct (~40 rows) as soon as the array is sized
	const quint32 nBodiesRaw = pf.array( root + 64, start );	// read the count first: it also sets start
	const quint32 nBodies = take( nBodiesRaw, start, 96, 40 );
	resize( item, "Num Bodies", "Bodies", nBodies );
	if ( NifItem * a = child( item, "Bodies" ) ) {
		for ( quint32 i = 0; i < nBodies && int( i ) < a->childCount(); i++ )
			body( a->child( int( i ) ), root, i );
	}

	// hknpMaterial (80 bytes): triggerType 16, dynamicFriction 18 (half), staticFriction 20, restitution 22,
	// friction/restitution combine policy 24/25, weldingTolerance 26 (half), maxContactImpulse 28
	const quint32 nMaterialsRaw = pf.array( root + 16, start );	// read the count first: it also sets start
	const quint32 nMaterials = take( nMaterialsRaw, start, 80, 9 );
	resize( item, "Num Materials", "Materials", nMaterials );
	if ( NifItem * a = child( item, "Materials" ) ) {
		for ( quint32 i = 0; i < nMaterials && int( i ) < a->childCount(); i++ ) {
			NifItem * m = a->child( int( i ) );
			const quint32 o = start + i * 80;
			put<float>( m, "Dynamic Friction", half( pf.get<quint16>( o + 18 ) ) );
			put<float>( m, "Static Friction", half( pf.get<quint16>( o + 20 ) ) );
			put<float>( m, "Restitution", half( pf.get<quint16>( o + 22 ) ) );
			put<quint32>( m, "Friction Combine Policy", pf.get<quint8>( o + 24 ) );
			put<quint32>( m, "Restitution Combine Policy", pf.get<quint8>( o + 25 ) );
			put<float>( m, "Welding Tolerance", half( pf.get<quint16>( o + 26 ) ) );
			put<float>( m, "Max Contact Impulse", pf.get<float>( o + 28 ) );
			put<quint32>( m, "Trigger Type", pf.get<quint8>( o + 16 ) );
		}
	}

	// hknpMotionProperties (64 bytes): gravityFactor 8, timeFactor 12, maxLinearSpeed 16, maxAngularSpeed 20,
	// linearDamping 24, angularDamping 28
	const quint32 nPropsRaw = pf.array( root + 32, start );	// read the count first: it also sets start
	const quint32 nProps = take( nPropsRaw, start, 64, 7 );
	resize( item, "Num Motion Properties", "Motion Properties", nProps );
	if ( NifItem * a = child( item, "Motion Properties" ) ) {
		for ( quint32 i = 0; i < nProps && int( i ) < a->childCount(); i++ ) {
			NifItem * m = a->child( int( i ) );
			const quint32 o = start + i * 64;
			put<float>( m, "Gravity Factor", pf.get<float>( o + 8 ) );
			put<float>( m, "Time Factor", pf.get<float>( o + 12 ) );
			put<float>( m, "Max Linear Speed", pf.get<float>( o + 16 ) );
			put<float>( m, "Max Angular Speed", pf.get<float>( o + 20 ) );
			put<float>( m, "Linear Damping", pf.get<float>( o + 24 ) );
			put<float>( m, "Angular Damping", pf.get<float>( o + 28 ) );
		}
	}

	// hknpMotionCinfo (112 bytes): motionPropertiesId 0 (u16), enableDeactivation 2, inverseMass 4, massFactor 8,
	// inverseInertiaLocal 32, centerOfMassWorld 48, orientation 64 (x, y, z, w)
	const quint32 nMotionsRaw = pf.array( root + 48, start );	// read the count first: it also sets start
	const quint32 nMotions = take( nMotionsRaw, start, 112, 8 );
	resize( item, "Num Motions", "Motions", nMotions );
	if ( NifItem * a = child( item, "Motions" ) ) {
		for ( quint32 i = 0; i < nMotions && int( i ) < a->childCount(); i++ ) {
			NifItem * m = a->child( int( i ) );
			const quint32 o = start + i * 112;
			const float invMass = pf.get<float>( o + 4 );
			put<float>( m, "Mass", invMass > 0.0f ? 1.0f / invMass : 0.0f );
			put<float>( m, "Mass Factor", pf.get<float>( o + 8 ) );
			put<Vector3>( m, "Inverse Inertia", pf.getVector3( o + 32 ) );
			put<Vector3>( m, "Center Of Mass", pf.getVector3( o + 48 ) );
			put<Quat>( m, "Orientation", Quat( pf.get<float>( o + 76 ), pf.get<float>( o + 64 ),
											   pf.get<float>( o + 68 ), pf.get<float>( o + 72 ) ) );
			put<quint32>( m, "Motion Properties ID", pf.get<quint16>( o ) );
			put<bool>( m, "Enable Deactivation", pf.get<quint8>( o + 2 ) != 0 );
		}
	}

	quint32 unused;
	put<quint32>( item, "Num Constraints", pf.array( root + 80, unused ) );
}

// Wireframes per bhkPhysicsSystem, keyed by the address of its "Binary Data". Each entry keeps a copy of
// that QByteArray, so the address stays unique while cached. Cleared when the scene is rebuilt.
QHash<const char *, System> & cache()
{
	static QHash<const char *, System> c;
	return c;
}

// The "Physics System" view each bhkPhysicsSystem block was last filled from (model, block number -> a copy of
// its Binary Data). Several collision objects can share one system; it is filled once, not once per object.
// Holding the copy keeps the data's address unique while recorded. Cleared with the scene.
QHash<QPair<const NifModel *, int>, QByteArray> & filledSystems()
{
	static QHash<QPair<const NifModel *, int>, QByteArray> s;
	return s;
}

}	// namespace

void HknpCollision::clearCache()
{
	cache().clear();
	filledSystems().clear();
}

void HknpCollision::updateViews( NifModel * nif, const QModelIndex & iObject )
{
	if ( !nif || !nif->isNiBlock( iObject, "bhkNPCollisionObject" ) )
		return;
	// "Havok Body", not "Body": the Skyrim collision code looks up a "Body" link on every collision object
	NifItem * bodyView = nif->getItem( iObject, "Havok Body" );
	const QModelIndex iSystem = nif->getBlockIndex( nif->getLink( iObject, "Data" ) );
	if ( !bodyView || !nif->isNiBlock( iSystem, "bhkPhysicsSystem" ) )
		return;
	NifItem * systemView = nif->getItem( iSystem, "Physics System" );

	// the packfile reader points into this buffer, so it must stay alive while filling
	const QByteArray data = nif->get<QByteArray>( iSystem, "Binary Data" );
	Packfile pf;
	if ( !pf.parse( data ) )
		return;
	const quint32 root = pf.findObject( "hknpPhysicsSystemData" );
	if ( root == quint32( -1 ) )
		return;

	// Filling the views is not an edit: in the Processing state, resizing their arrays does not emit dataChanged,
	// so the file is not marked as modified (no asterisk in the title, no "save changes?" prompt).
	nif->setState( BaseModel::Processing );
	ViewFiller fill( nif, pf );
	fill.body( bodyView, root, nif->get<quint32>( iObject, "Body ID" ) );
	// the shared system view: only when it has not been filled from this exact data yet
	const QPair<const NifModel *, int> key( nif, nif->getBlockNumber( iSystem ) );
	if ( systemView && filledSystems().value( key ).constData() != data.constData() ) {
		ViewFiller( nif, pf ).system( systemView, root );
		filledSystems().insert( key, data );
	}
	nif->restoreState();
}

const HknpCollision::Body * HknpCollision::getBody( const QByteArray & data, quint32 bodyID )
{
	if ( data.isEmpty() )
		return nullptr;

	auto i = cache().find( data.constData() );
	if ( i == cache().end() ) {
		System sys;
		sys.data = data;
		loadSystem( sys );
		i = cache().insert( data.constData(), sys );
	}
	if ( bodyID >= quint32( i->bodies.size() ) )
		return nullptr;
	return &( i->bodies.at( bodyID ) );
}

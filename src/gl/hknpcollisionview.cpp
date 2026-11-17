#include "hknpcollision.h"

#include "gl/hknplayout.h"
#include "gl/hkpackfile.h"
#include "model/nifmodel.h"

#include <QHash>
#include <QPair>

#include <cstring>

//! @file hknpcollisionview.cpp Read-only block tree views of Fallout 4 collision data

/*
 * Fills the abstract nif.xml structs hknpViewBody ("Havok Body" in bhkNPCollisionObject) and hknpViewPhysicsSystem
 * ("Physics System" in bhkPhysicsSystem) from the Havok packfile in bhkPhysicsSystem's Binary Data. Abstract items
 * are shown in the block tree but never loaded or saved, and BaseModel::isReadOnlyView() makes them read-only.
 * Values are shown as stored, in Havok units.
 */

using namespace HknpLayout;

namespace
{

//! Writes decoded values into the view items of the model
class ViewWriter
{
public:
	explicit ViewWriter( NifModel * n ) : nif( n ) {}

	NifItem * child( NifItem * parent, const char * name ) const
	{
		return parent ? nif->getItem( parent, QLatin1StringView( name ) ) : nullptr;
	}
	//! Sets a field if the struct has it (fields switched off by their condition are skipped)
	template <typename T> void put( NifItem * parent, const char * name, const T & v ) const
	{
		NifItem::set<T>( child( parent, name ), v );
	}
	//! Re-evaluates the conditions of a struct's fields after a value they depend on was set
	static void refreshConditions( NifItem * item )
	{
		if ( item ) {
			for ( auto c : item->children() )
				c->invalidateCondition();
		}
	}
	/*! Sets an array's count and resizes the array. Arrays the struct does not have, or that are switched off by
	 *  their condition, are skipped (the model does not return those, and asking for them by name reports an error).
	 */
	void resize( NifItem * parent, const char * count, const char * array, quint32 n ) const
	{
		if ( !parent || !child( parent, array ) )
			return;
		put<quint32>( parent, count, n );
		refreshConditions( parent );
		nif->updateArraySize( parent, array );
	}

	//! Empties a shape view: unknown type, no arrays (used when a body cannot be decoded)
	void clearShape( NifItem * item ) const
	{
		put<quint32>( item, "Shape Type", 0 );
		put<bool>( item, "Has Mass Properties", false );
		for ( const char * count : { "Vert Count", "Plane Count", "Face Count", "Num Instances" } )
			put<quint32>( item, count, 0 );
		refreshConditions( item );
	}
	//! Empties a body view
	void clearBody( NifItem * item ) const
	{
		for ( const char * name : { "Body ID", "Collision Filter", "Layer", "Flags", "Motion ID", "Material ID", "Quality ID" } )
			put<quint32>( item, name, 0 );
		put<float>( item, "Mass", 0.0f );
		clearShape( child( item, "Shape" ) );
	}
	//! Empties a physics system view
	void clearSystem( NifItem * item ) const
	{
		resize( item, "Num Bodies", "Bodies", 0 );
		resize( item, "Num Materials", "Materials", 0 );
		resize( item, "Num Motion Properties", "Motion Properties", 0 );
		resize( item, "Num Motions", "Motions", 0 );
		put<quint32>( item, "Num Constraints", 0 );
	}

protected:
	NifModel * nif;
};

//! Fills the views from a parsed packfile
class ViewFiller : public ViewWriter
{
public:
	ViewFiller( NifModel * n, const HkPackfile & p ) : ViewWriter( n ), pf( p ) {}

	void system( NifItem * item, quint32 root );
	void body( NifItem * item, quint32 root, quint32 bodyID );

private:
	static constexpr int maxDepth = 8;
	static constexpr quint32 maxArray = 65536;

	/*! Number of array elements to show: the count from the file, limited to what the data holds and to the
	 *  remaining row budget (each element costs @p rowsEach rows). A malformed file can repeat a large shape in
	 *  many bodies or instances; the budget keeps the whole view bounded. Real files stay far below it.
	 */
	quint32 take( const HkSpan & span, quint32 stride, int rowsEach )
	{
		const quint32 n = std::min( pf.clamp( span, stride ).count, maxArray );
		const qsizetype affordable = std::max< qsizetype >( rowBudget, 0 ) / rowsEach;
		const quint32 taken = quint32( std::min< qsizetype >( n, affordable ) );
		rowBudget -= qsizetype( taken ) * rowsEach;
		return taken;
	}
	//! hkHalf: the high 16 bits of a float
	static float half( quint16 h )
	{
		const quint32 bits = quint32( h ) << 16;
		float f;
		std::memcpy( &f, &bits, 4 );
		return f;
	}
	//! hkPackedVector3: three int16 values, and a scale (the high half of a float) in the fourth
	Vector3 packedVector( quint32 offset ) const
	{
		const quint32 bits = quint32( pf.get<quint16>( offset + 6 ) ) << 16;
		float scale;
		std::memcpy( &scale, &bits, 4 );
		scale *= 65536.0f;
		return Vector3( pf.get<qint16>( offset ) * scale, pf.get<qint16>( offset + 2 ) * scale,
						pf.get<qint16>( offset + 4 ) * scale );
	}

	quint32 shapeType( quint32 shape ) const;
	void shape( NifItem * item, quint32 shape, int depth );
	void convexData( NifItem * item, quint32 shape, bool polytope );
	void compoundInstances( NifItem * item, quint32 shape, int depth );
	void massProperties( NifItem * item, quint32 shape );

	const HkPackfile & pf;
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

void ViewFiller::massProperties( NifItem * item, quint32 shape )
{
	// the hknpShapeMassProperties entry of the shape's hkRefCountedProperties
	quint32 props = 0, massProps = 0;
	bool found = false;
	if ( pf.pointer( shape + shapeProperties, props ) ) {
		const HkSpan entries = pf.array( props );
		for ( quint32 i = 0; i < entries.count && i < 64 && !found; i++ ) {
			const quint32 e = entries.start + i * propertyEntrySize;
			found = ( pf.get<quint16>( e + propertyEntryKey ) == massPropertiesKey
					  && pf.pointer( e + propertyEntryObject, massProps ) );
		}
	}
	put<bool>( item, "Has Mass Properties", found );
	refreshConditions( item );
	if ( !found )
		return;
	NifItem * m = child( item, "Mass Properties" );
	const quint32 c = massProps + HknpLayout::massProperties;
	put<float>( m, "Volume", pf.get<float>( c + massPropertiesVolume ) );
	put<float>( m, "Mass", pf.get<float>( c + massPropertiesMass ) );
	put<Vector3>( m, "Center Of Mass", packedVector( c + massPropertiesCenterOfMass ) );
	put<Vector3>( m, "Inertia", packedVector( c + massPropertiesInertia ) );
}

void ViewFiller::convexData( NifItem * item, quint32 shape, bool polytope )
{
	const HkSpan vertices = pf.relArray( shape + convexVertices );
	const quint32 nv = take( vertices, vertexSize, 1 );
	resize( item, "Vert Count", "Vertices", nv );
	if ( NifItem * a = child( item, "Vertices" ) ) {
		for ( quint32 i = 0; i < nv && int( i ) < a->childCount(); i++ )
			NifItem::set<Vector3>( a->child( int( i ) ), pf.getVector3( vertices.start + i * vertexSize ) );
	}
	if ( !polytope )
		return;

	const HkSpan planes = pf.relArray( shape + polytopePlanes );
	const quint32 np = take( planes, planeSize, 1 );
	resize( item, "Plane Count", "Planes", np );
	if ( NifItem * a = child( item, "Planes" ) ) {
		for ( quint32 i = 0; i < np && int( i ) < a->childCount(); i++ )
			NifItem::set<Vector4>( a->child( int( i ) ), pf.getVector4( planes.start + i * planeSize ) );
	}

	const HkSpan faces = pf.relArray( shape + polytopeFaces );
	const quint32 nf = take( faces, faceSize, 3 );
	resize( item, "Face Count", "Faces", nf );
	if ( NifItem * a = child( item, "Faces" ) ) {
		for ( quint32 i = 0; i < nf && int( i ) < a->childCount(); i++ ) {
			NifItem * f = a->child( int( i ) );
			const quint32 o = faces.start + i * faceSize;
			put<quint16>( f, "First Index", pf.get<quint16>( o + faceFirstIndex ) );
			put<quint8>( f, "Num Indices", pf.get<quint8>( o + faceNumIndices ) );
		}
	}
}

void ViewFiller::compoundInstances( NifItem * item, quint32 shape, int depth )
{
	if ( !child( item, "Instances" ) )
		return;		// nested compounds are not expanded (hknpViewChildShape has no instances)
	const HkSpan instances = pf.array( shape + HknpLayout::compoundInstances );
	// each instance creates its fields and a nested child shape struct (~30 rows)
	const quint32 n = take( instances, instanceSize, 30 );
	resize( item, "Num Instances", "Instances", n );
	NifItem * a = child( item, "Instances" );
	for ( quint32 i = 0; a && i < n && int( i ) < a->childCount(); i++ ) {
		NifItem * inst = a->child( int( i ) );
		const quint32 o = instances.start + i * instanceSize;
		// the rotation is stored as 3 columns; Matrix takes rows
		const Vector3 cx = pf.getVector3( o + instanceRotation );
		const Vector3 cy = pf.getVector3( o + instanceRotation + 16 );
		const Vector3 cz = pf.getVector3( o + instanceRotation + 32 );
		const float rows[9] = { cx[0], cy[0], cz[0], cx[1], cy[1], cz[1], cx[2], cy[2], cz[2] };
		put<quint16>( inst, "Instance ShapeTag", pf.get<quint16>( o + instanceShapeTag ) );
		put<Vector3>( inst, "Instance Scale", pf.getVector3( o + instanceScale ) );
		put<Vector3>( inst, "Instance Translate", pf.getVector3( o + instanceTranslation ) );
		put<Matrix>( inst, "Instance Rotate", Matrix( rows ) );
		quint32 childShape;
		if ( pf.pointer( o + instanceShape, childShape ) )
			this->shape( child( inst, "Shape" ), childShape, depth + 1 );
		else
			clearShape( child( inst, "Shape" ) );	// free slot
	}
}

void ViewFiller::shape( NifItem * item, quint32 shape, int depth )
{
	if ( !item )
		return;
	rowBudget -= 25;	// the shape's own fields
	const quint32 type = ( depth <= maxDepth ? shapeType( shape ) : 0 );
	put<quint32>( item, "Shape Type", type );
	put<quint32>( item, "UserData", quint32( pf.get<quint64>( shape + shapeUserData ) ) );
	put<float>( item, "Convex Radius", pf.get<float>( shape + shapeConvexRadius ) );
	refreshConditions( item );

	// zero the counts, so arrays that are hidden for this shape type stay empty if they are shown later
	for ( const char * count : { "Vert Count", "Plane Count", "Face Count", "Num Instances" } )
		put<quint32>( item, count, 0 );

	switch ( type ) {
	case 1:		// Convex Polytope Shape
	case 2:		// Convex Shape
		convexData( item, shape, type == 1 );
		break;
	case 3:		// Capsule Shape (radius = convex radius)
		put<Vector3>( item, "Point A", pf.getVector3( shape + capsulePointA ) );
		put<Vector3>( item, "Point B", pf.getVector3( shape + capsulePointB ) );
		break;
	case 4: {	// Sphere Shape: centre = vertices[0], radius = convex radius
		const HkSpan verts = pf.relArray( shape + convexVertices );
		put<Vector3>( item, "Center", verts.count ? pf.getVector3( verts.start ) : Vector3() );
		break;
	}
	case 5: {	// Scaled Convex Shape
		put<Vector3>( item, "Scale", pf.getVector3( shape + scaledScale ) );
		put<Vector3>( item, "Translation", pf.getVector3( shape + scaledTranslation ) );
		quint32 core;
		if ( NifItem * c = child( item, "Core Shape" ); c && pf.pointer( shape + scaledCoreShape, core ) )
			this->shape( c, core, depth + 1 );
		break;
	}
	case 6:		// Static Compound Shape
	case 7:		// Dynamic Compound Shape
		compoundInstances( item, shape, depth );
		break;
	case 8: {	// Compressed Mesh Shape: summary only (a mesh can have hundreds of thousands of triangles)
		quint32 data;
		if ( child( item, "Num Sections" ) && pf.pointer( shape + compressedMeshData, data ) ) {
			const quint32 tree = data + meshTree;
			put<quint32>( item, "Num Sections", pf.array( tree + treeSections ).count );
			put<quint32>( item, "Num Primitives", pf.array( tree + treePrimitives ).count );
			put<Vector3>( item, "Bounds Min", pf.getVector3( tree + treeDomainMin ) );
			put<Vector3>( item, "Bounds Max", pf.getVector3( tree + treeDomainMax ) );
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
	const HkSpan bodies = pf.clamp( pf.array( root + systemBodies ), bodySize );
	const HkSpan motions = pf.clamp( pf.array( root + systemMotions ), motionSize );
	if ( bodyID >= bodies.count ) {
		clearBody( item );	// e.g. Body ID edited to a value past the end: don't keep showing the old body
		return;
	}
	const quint32 b = bodies.start + bodyID * bodySize;
	const quint32 filter = pf.get<quint32>( b + bodyFilterInfo );
	const quint32 motionID = pf.get<quint32>( b + bodyMotionId );
	put<quint32>( item, "Body ID", bodyID );
	put<quint32>( item, "Collision Filter", filter );
	put<quint32>( item, "Layer", filter & 0x7F );
	put<quint32>( item, "Flags", pf.get<quint32>( b + bodyFlags ) );
	put<quint32>( item, "Motion ID", motionID );
	put<quint32>( item, "Material ID", pf.get<quint16>( b + bodyMaterialId ) );
	put<quint32>( item, "Quality ID", pf.get<quint8>( b + bodyQualityId ) );
	float mass = 0.0f;
	if ( motionID < motions.count ) {
		const float invMass = pf.get<float>( motions.start + motionID * motionSize + motionInverseMass );
		if ( invMass > 0.0f )
			mass = 1.0f / invMass;
	}
	put<float>( item, "Mass", mass );
	quint32 s;
	if ( pf.pointer( b + bodyShape, s ) )
		shape( child( item, "Shape" ), s, 0 );
	else
		clearShape( child( item, "Shape" ) );
}

void ViewFiller::system( NifItem * item, quint32 root )
{
	if ( !item )
		return;

	// each body creates a body struct with a nested shape struct (~40 rows) as soon as the array is sized
	const quint32 nBodies = take( pf.array( root + systemBodies ), bodySize, 40 );
	resize( item, "Num Bodies", "Bodies", nBodies );
	if ( NifItem * a = child( item, "Bodies" ) ) {
		for ( quint32 i = 0; i < nBodies && int( i ) < a->childCount(); i++ )
			body( a->child( int( i ) ), root, i );
	}

	const HkSpan materials = pf.array( root + systemMaterials );
	const quint32 nMaterials = take( materials, materialSize, 9 );
	resize( item, "Num Materials", "Materials", nMaterials );
	if ( NifItem * a = child( item, "Materials" ) ) {
		for ( quint32 i = 0; i < nMaterials && int( i ) < a->childCount(); i++ ) {
			NifItem * m = a->child( int( i ) );
			const quint32 o = materials.start + i * materialSize;
			put<float>( m, "Dynamic Friction", half( pf.get<quint16>( o + materialDynamicFriction ) ) );
			put<float>( m, "Static Friction", half( pf.get<quint16>( o + materialStaticFriction ) ) );
			put<float>( m, "Restitution", half( pf.get<quint16>( o + materialRestitution ) ) );
			put<quint32>( m, "Friction Combine Policy", pf.get<quint8>( o + materialFrictionPolicy ) );
			put<quint32>( m, "Restitution Combine Policy", pf.get<quint8>( o + materialRestitutionPolicy ) );
			put<float>( m, "Welding Tolerance", half( pf.get<quint16>( o + materialWeldingTolerance ) ) );
			put<float>( m, "Max Contact Impulse", pf.get<float>( o + materialMaxContactImpulse ) );
			put<quint32>( m, "Trigger Type", pf.get<quint8>( o + materialTriggerType ) );
		}
	}

	const HkSpan props = pf.array( root + systemMotionProperties );
	const quint32 nProps = take( props, motionPropertiesSize, 7 );
	resize( item, "Num Motion Properties", "Motion Properties", nProps );
	if ( NifItem * a = child( item, "Motion Properties" ) ) {
		for ( quint32 i = 0; i < nProps && int( i ) < a->childCount(); i++ ) {
			NifItem * m = a->child( int( i ) );
			const quint32 o = props.start + i * motionPropertiesSize;
			put<float>( m, "Gravity Factor", pf.get<float>( o + motionPropertiesGravityFactor ) );
			put<float>( m, "Time Factor", pf.get<float>( o + motionPropertiesTimeFactor ) );
			put<float>( m, "Max Linear Speed", pf.get<float>( o + motionPropertiesMaxLinearSpeed ) );
			put<float>( m, "Max Angular Speed", pf.get<float>( o + motionPropertiesMaxAngularSpeed ) );
			put<float>( m, "Linear Damping", pf.get<float>( o + motionPropertiesLinearDamping ) );
			put<float>( m, "Angular Damping", pf.get<float>( o + motionPropertiesAngularDamping ) );
		}
	}

	const HkSpan motions = pf.array( root + systemMotions );
	const quint32 nMotions = take( motions, motionSize, 8 );
	resize( item, "Num Motions", "Motions", nMotions );
	if ( NifItem * a = child( item, "Motions" ) ) {
		for ( quint32 i = 0; i < nMotions && int( i ) < a->childCount(); i++ ) {
			NifItem * m = a->child( int( i ) );
			const quint32 o = motions.start + i * motionSize;
			const float invMass = pf.get<float>( o + motionInverseMass );
			const quint32 q = o + motionOrientation;	// x, y, z, w; Quat takes w first
			put<float>( m, "Mass", invMass > 0.0f ? 1.0f / invMass : 0.0f );
			put<float>( m, "Mass Factor", pf.get<float>( o + motionMassFactor ) );
			put<Vector3>( m, "Inverse Inertia", pf.getVector3( o + motionInverseInertia ) );
			put<Vector3>( m, "Center Of Mass", pf.getVector3( o + motionCenterOfMass ) );
			put<Quat>( m, "Orientation", Quat( pf.get<float>( q + 12 ), pf.get<float>( q ),
											   pf.get<float>( q + 4 ), pf.get<float>( q + 8 ) ) );
			put<quint32>( m, "Motion Properties ID", pf.get<quint16>( o + motionPropertiesId ) );
			put<bool>( m, "Enable Deactivation", pf.get<quint8>( o + motionEnableDeactivation ) != 0 );
		}
	}

	put<quint32>( item, "Num Constraints", pf.array( root + systemConstraints ).count );
}

// The "Physics System" view each bhkPhysicsSystem block was last filled from (model, block number -> a copy of its
// Binary Data). Several collision objects can share one system: it is filled once, not once per object. Holding the
// copy keeps the data's address unique while recorded. Cleared with the scene (HknpCollision::clearCache).
QHash<QPair<const NifModel *, int>, QByteArray> & filledSystems()
{
	static QHash<QPair<const NifModel *, int>, QByteArray> s;
	return s;
}

}	// namespace

void HknpCollision::Detail::clearViewRecords()
{
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

	// Filling the views is not an edit: in the Processing state, resizing their arrays does not emit dataChanged,
	// so the file is not marked as modified (no asterisk in the title, no "save changes?" prompt).
	nif->setState( BaseModel::Processing );

	const QByteArray data = nif->get<QByteArray>( iSystem, "Binary Data" );
	const QPair<const NifModel *, int> key( nif, nif->getBlockNumber( iSystem ) );
	const bool systemIsCurrent = ( filledSystems().value( key ).constData() == data.constData() );
	quint32 root = 0;
	if ( const HkPackfile * pf = Detail::parsedSystem( data, root ) ) {
		ViewFiller( nif, *pf ).body( bodyView, root, nif->get<quint32>( iObject, "Body ID" ) );
		if ( systemView && !systemIsCurrent )
			ViewFiller( nif, *pf ).system( systemView, root );
	} else {
		// the data cannot be decoded (e.g. after an edit): don't keep showing what it held before
		ViewWriter writer( nif );
		writer.clearBody( bodyView );
		if ( !systemIsCurrent )
			writer.clearSystem( systemView );
	}
	if ( systemView )
		filledSystems().insert( key, data );

	nif->restoreState();
}

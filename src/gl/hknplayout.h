#ifndef HKNPLAYOUT_H
#define HKNPLAYOUT_H

#include <QtGlobal>

//! @file hknplayout.h Byte offsets of the Havok 2014 (hknp) classes in Fallout 4 collision packfiles

/*
 * Fallout 4 packfiles do not describe their own class layouts (the __types__ section is empty), so the layouts
 * the reader needs are listed here, once, for both the wireframe builder and the block tree view.
 *
 * Offsets are for the 64-bit layout ("hk_2014.1.0-r1"). They come from HkxPack-Plus's 2014 class definitions
 * and were checked against base-game meshes; corrections found while checking:
 *  - hkRelArray = u16 count, then a u16 offset measured from the hkRelArray field itself;
 *  - a compressed mesh section's page byte selects a block of 65536 shared vertices.
 * Research notes: fo4-exporter-re research/collision/README.md.
 */
namespace HknpLayout
{
	// hknpPhysicsSystemData: hkArrays of the system's contents
	constexpr quint32 systemMaterials = 16;			// hknpMaterial[]
	constexpr quint32 systemMotionProperties = 32;	// hknpMotionProperties[]
	constexpr quint32 systemMotions = 48;			// hknpMotionCinfo[]
	constexpr quint32 systemBodies = 64;			// hknpBodyCinfo[]
	constexpr quint32 systemConstraints = 80;		// hknpConstraintCinfo[]

	// hknpBodyCinfo. Its position (48) and orientation (64) are not used: they record where the body was at
	// export, while the shape is already in the space of the collision object's node.
	constexpr quint32 bodySize = 96;
	constexpr quint32 bodyShape = 0;				// hknpShape*
	constexpr quint32 bodyMotionId = 12;			// u32, 0x7FFFFFFF = no motion (static)
	constexpr quint32 bodyQualityId = 16;			// u8
	constexpr quint32 bodyMaterialId = 18;			// u16
	constexpr quint32 bodyFilterInfo = 20;			// u32, layer in the low 7 bits
	constexpr quint32 bodyFlags = 24;				// u32, hknpBody flags
	constexpr quint32 noMotion = 0x7FFFFFFF;

	// hknpMaterial (frictions, restitution and welding tolerance are hkHalf: the high 16 bits of a float)
	constexpr quint32 materialSize = 80;
	constexpr quint32 materialTriggerType = 16;			// u8
	constexpr quint32 materialDynamicFriction = 18;
	constexpr quint32 materialStaticFriction = 20;
	constexpr quint32 materialRestitution = 22;
	constexpr quint32 materialFrictionPolicy = 24;		// u8
	constexpr quint32 materialRestitutionPolicy = 25;	// u8
	constexpr quint32 materialWeldingTolerance = 26;
	constexpr quint32 materialMaxContactImpulse = 28;	// float

	// hknpMotionProperties (floats)
	constexpr quint32 motionPropertiesSize = 64;
	constexpr quint32 motionPropertiesGravityFactor = 8;
	constexpr quint32 motionPropertiesTimeFactor = 12;
	constexpr quint32 motionPropertiesMaxLinearSpeed = 16;
	constexpr quint32 motionPropertiesMaxAngularSpeed = 20;
	constexpr quint32 motionPropertiesLinearDamping = 24;
	constexpr quint32 motionPropertiesAngularDamping = 28;

	// hknpMotionCinfo
	constexpr quint32 motionSize = 112;
	constexpr quint32 motionPropertiesId = 0;		// u16
	constexpr quint32 motionEnableDeactivation = 2;	// u8
	constexpr quint32 motionInverseMass = 4;		// float
	constexpr quint32 motionMassFactor = 8;			// float, mass / shape volume
	constexpr quint32 motionInverseInertia = 32;	// hkVector4
	constexpr quint32 motionCenterOfMass = 48;		// hkVector4
	constexpr quint32 motionOrientation = 64;		// hkQuaternion (x, y, z, w), principal axes

	// hknpShape (all shapes)
	constexpr quint32 shapeConvexRadius = 20;		// float
	constexpr quint32 shapeUserData = 24;			// u64, the Bethesda material ID in the low 32 bits
	constexpr quint32 shapeProperties = 32;			// hkRefCountedProperties*

	// hkRefCountedProperties: hkArray of entries at 0
	constexpr quint32 propertyEntrySize = 16;
	constexpr quint32 propertyEntryObject = 0;		// hkReferencedObject*
	constexpr quint32 propertyEntryKey = 8;			// u16
	constexpr quint16 massPropertiesKey = 0xF100;	// the entry holding hknpShapeMassProperties

	// hknpShapeMassProperties: hkCompressedMassProperties at 16 (unit density; see the mass rules in the notes)
	constexpr quint32 massProperties = 16;
	constexpr quint32 massPropertiesCenterOfMass = 0;	// hkPackedVector3
	constexpr quint32 massPropertiesInertia = 8;		// hkPackedVector3, principal inertia
	constexpr quint32 massPropertiesMass = 24;			// float
	constexpr quint32 massPropertiesVolume = 28;		// float

	// hknpConvexShape / hknpConvexPolytopeShape (hkRelArrays)
	constexpr quint32 convexVertices = 48;			// hkVector4[]; w holds the vertex index, not a coordinate
	constexpr quint32 polytopePlanes = 64;			// hkVector4[] (normal, distance), one per face, inner hull
	constexpr quint32 polytopeFaces = 68;			// face[]
	constexpr quint32 polytopeIndices = 72;			// u8[]
	constexpr quint32 vertexSize = 16;
	constexpr quint32 planeSize = 16;
	constexpr quint32 faceSize = 4;
	constexpr quint32 faceFirstIndex = 0;			// u16
	constexpr quint32 faceNumIndices = 2;			// u8

	// hknpCapsuleShape (radius = convex radius); hknpSphereShape: centre = vertices[0], radius = convex radius
	constexpr quint32 capsulePointA = 80;
	constexpr quint32 capsulePointB = 96;

	// hknpScaledConvexShape: vertex' = vertex * scale + translation
	constexpr quint32 scaledCoreShape = 48;			// hknpShape* (hknpDecoratorShape::coreShape)
	constexpr quint32 scaledScale = 64;
	constexpr quint32 scaledTranslation = 80;

	// hknpStatic/DynamicCompoundShape: hkArray of hknpShapeInstance at 96 (null shape = free slot)
	constexpr quint32 compoundInstances = 96;
	constexpr quint32 instanceSize = 128;
	constexpr quint32 instanceRotation = 0;			// 3 columns of hkVector4 at 0, 16, 32
	constexpr quint32 instanceTranslation = 48;
	constexpr quint32 instanceScale = 64;
	constexpr quint32 instanceShape = 80;			// hknpShape*
	constexpr quint32 instanceShapeTag = 88;		// u16

	// hknpCompressedMeshShape: data at 96 -> hknpCompressedMeshShapeData, whose mesh tree
	// (hknpCompressedMeshShapeTree, an hkcdStaticMeshTree) starts at 16
	constexpr quint32 compressedMeshData = 96;
	constexpr quint32 meshTree = 16;
	constexpr quint32 treeDomainMin = 16;			// AABB of the whole mesh
	constexpr quint32 treeDomainMax = 32;
	constexpr quint32 treeSections = 64;			// section[]
	constexpr quint32 treePrimitives = 80;			// u8[4][]: a triangle if [2] == [3], else a quad
	constexpr quint32 treeSharedIndices = 96;		// u16[]
	constexpr quint32 treePackedVertices = 112;		// u32[]: 11/11/10 bits, decoded with the section codec
	constexpr quint32 treeSharedVertices = 128;		// u64[]: 21/21/22 bits across the domain
	constexpr quint32 sectionSize = 96;
	constexpr quint32 sectionCodecOffset = 48;		// xyz
	constexpr quint32 sectionCodecScale = 60;		// xyz
	constexpr quint32 sectionFirstPacked = 72;		// u32
	constexpr quint32 sectionSharedVertices = 76;	// u32: count in the low 8 bits, offset above
	constexpr quint32 sectionPrimitives = 80;		// u32: count in the low 8 bits, offset above
	constexpr quint32 sectionNumPacked = 88;		// u8
	constexpr quint32 sectionPage = 92;				// u8: shared vertex index = page * 65536 + u16 index
	constexpr quint32 primitiveSize = 4;
}

#endif

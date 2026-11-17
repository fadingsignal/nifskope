#ifndef HKNPCOLLISION_H
#define HKNPCOLLISION_H

#include "data/niftypes.h"

#include <QByteArray>
#include <QVector>

//! @file hknpcollision.h Wireframes for Fallout 4 collision (Havok packfiles in bhkPhysicsSystem)

namespace HknpCollision
{
	//! One body of an hknpPhysicsSystemData, ready to draw
	struct Body
	{
		//! Line list (pairs of points) in Havok units, in the space of the collision object's node
		QVector<Vector3> lines;
		//! hknpBodyCinfo::collisionFilterInfo (the low 7 bits are the collision layer)
		quint32 filterInfo = 0;
	};

	/*! Returns the wireframe of body @p bodyID in the packfile held by a bhkPhysicsSystem,
	 *  or nullptr if the data cannot be read or there is no such body.
	 *  Results are cached per QByteArray, so this is cheap to call every frame.
	 */
	const Body * getBody( const QByteArray & data, quint32 bodyID );

	//! Frees all cached wireframes; call when the scene is rebuilt (e.g. a file is loaded or closed)
	void clearCache();
}

#endif

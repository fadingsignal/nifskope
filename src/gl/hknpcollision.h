#ifndef HKNPCOLLISION_H
#define HKNPCOLLISION_H

#include "data/niftypes.h"

#include <QByteArray>
#include <QModelIndex>
#include <QVector>

class HkPackfile;
class NifModel;

//! @file hknpcollision.h Fallout 4 collision (Havok packfiles in bhkPhysicsSystem): wireframes and block tree views

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

	//! Frees all cached data; call when the scene is rebuilt (e.g. a file is loaded or closed)
	void clearCache();

	/*! Fills the read-only views of a bhkNPCollisionObject (its "Havok Body") and of the bhkPhysicsSystem it
	 *  links to ("Physics System"). Both are abstract nif.xml fields: shown in the block tree, never saved.
	 *  If the data cannot be decoded, the views are emptied. Does nothing for files without these fields.
	 *  Implemented in hknpcollisionview.cpp.
	 */
	void updateViews( NifModel * nif, const QModelIndex & iObject );

	// Shared by hknpcollision.cpp and hknpcollisionview.cpp
	namespace Detail
	{
		/*! The parsed packfile of a bhkPhysicsSystem's Binary Data (parsed once, cached until clearCache()), and the
		 *  offset of its hknpPhysicsSystemData in @p root. nullptr if it cannot be read. The pointer is valid until
		 *  the next call that adds to the cache (getBody or parsedSystem with new data) or clearCache().
		 */
		const HkPackfile * parsedSystem( const QByteArray & data, quint32 & root );
		//! Forgets which physics system views have been filled (hknpcollisionview.cpp); called by clearCache()
		void clearViewRecords();
	}
}

#endif

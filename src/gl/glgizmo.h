/***** BEGIN LICENSE BLOCK *****

BSD License

Copyright (c) 2005-2015, NIF File Format Library and Tools
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The name of the NIF File Format Library and Tools project may not be
   used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***** END LICENCE BLOCK *****/

#ifndef GLGIZMO_H
#define GLGIZMO_H

#include "data/niftypes.h"

#include <QPointF>
#include <QSize>


//! @file glgizmo.h TransformGizmo

class Scene;

/*! A move/rotate/scale gizmo whose handles are always aligned to the world (NIF root) axes.
 *
 * The gizmo only deals with world space. Converting the drag result into the
 * local space of the object being transformed is left to the caller.
 */
class TransformGizmo final
{
public:
	enum Mode
	{
		Translate,
		Rotate,
		Scale    //!< Uniform only, NIF transforms have a single scale value
	};

	enum Handle
	{
		HandleNone = -1,
		// Axis arrows (Translate), rings (Rotate) or arms (Scale), value is the axis index
		HandleX = 0,
		HandleY = 1,
		HandleZ = 2,
		// Plane handles (Translate only), (value - 3) is the axis index of the plane normal
		HandleYZ = 3,
		HandleXZ = 4,
		HandleXY = 5,
		// Free move in the view plane (Translate), rotation around the view axis (Rotate) or center box (Scale)
		HandleScreen = 6
	};

	void setMode( Mode m ) { mode = m; dragHandle = HandleNone; }
	Mode getMode() const { return mode; }

	/*! Stores the camera used for the last rendered frame
	 *
	 * @param view			World to eye transform
	 * @param projection	Projection matrix (column-major, as passed to the shaders)
	 * @param widgetSize	Viewport size in the units of the mouse positions
	 */
	void setCamera( const Transform & view, const Matrix4 & projection, const QSize & widgetSize );
	//! Sets the gizmo position in world space
	void setOrigin( const Vector3 & origin );

	//! Returns the handle under the widget position, if any
	Handle hitTest( const QPointF & pos ) const;

	/*! Draws the gizmo with the scene's line and triangle helpers
	 *
	 * @param pixelScale	Multiplier for line widths and point sizes (high DPI)
	 */
	void draw( Scene * scene, Handle hover, Handle active, float pixelScale ) const;

	//! Starts dragging a handle, returns false if the handle cannot be dragged from this view
	bool beginDrag( Handle handle, const QPointF & pos );
	//! Translate: returns the world space offset from the drag start
	Vector3 dragDelta( const QPointF & pos );
	//! Rotate: returns the world space rotation from the drag start, optionally snapped to snapDegrees
	Matrix dragRotation( const QPointF & pos, float snapDegrees = 0 );
	//! Rotate: the current drag angle in degrees
	float dragAngle() const { return float( angle / PI * 180.0 ); }
	//! Scale: returns the uniform scale factor from the drag start, optionally snapped to multiples of snapStep
	float dragScale( const QPointF & pos, float snapStep = 0 );

private:
	//! Projects a world point to widget coordinates, returns false if behind the camera
	bool project( const Vector3 & world, QPointF & out ) const;
	//! Creates a world space ray through the widget position
	void ray( const QPointF & pos, Vector3 & origin, Vector3 & dir ) const;
	//! Unit vector from the gizmo origin towards the camera
	Vector3 toViewer() const;
	//! World space length of the gizmo arms
	float scale() const;
	//! True if the axis is not pointing (almost) straight at the camera
	bool axisVisible( int axis, float len ) const;
	//! True if the plane is not seen (almost) edge-on
	bool planeVisible( int normal ) const;

	//! The rotation axis of a ring handle
	Vector3 ringAxis( Handle handle ) const;
	//! Two orthonormal vectors spanning the plane of a ring handle
	void ringBasis( Handle handle, Vector3 & u, Vector3 & v ) const;
	//! World radius of a ring handle
	float ringRadius( Handle handle, float len ) const;
	//! Rotate: angle of the mouse ray hit on the ring plane, relative to the grabbed point
	bool planeAngle( const QPointF & pos, float & out ) const;

	Handle hitTestTranslate( const QPointF & pos ) const;
	Handle hitTestRotate( const QPointF & pos ) const;
	Handle hitTestScale( const QPointF & pos ) const;
	void drawTranslate( Scene * scene, Handle hover, Handle active, float ps ) const;
	void drawRotate( Scene * scene, Handle hover, Handle active, float ps ) const;
	void drawScale( Scene * scene, Handle hover, Handle active, float ps ) const;
	bool beginTranslate( Handle handle, const QPointF & pos );
	bool beginRotate( Handle handle, const QPointF & pos );
	bool beginScale( Handle handle, const QPointF & pos );

	Mode mode = Translate;

	Transform view;
	double viewProj[16];    //!< Projection * view, column-major
	double invViewProj[16]; //!< Inverse of viewProj
	bool perspective = true;
	QSize widget = { 1, 1 };

	Vector3 origin;

	Handle dragHandle = HandleNone;
	Vector3 dragOrigin;
	QPointF dragPress;

	// Translate
	Vector3 dragNormal;
	Vector3 dragStart;
	float dragParam = 0;
	Vector3 lastDelta;

	// Rotate
	Vector3 dragAxis;
	Vector3 dragRadial;    //!< Unit vector from the origin to the grabbed point on the ring
	bool dragPlanar = true; //!< Mouse ray intersected with the ring plane, otherwise linear along the ring tangent
	float lastPlaneAngle = 0;
	float rawAngle = 0;
	QPointF tangentDir;
	float pixelsPerRadian = 1;
	QPointF sideDir;      //!< Edge-on ring grabbed at its tip: circular motion around the center
	float sidePixelsPerRadian = 1;
	float angle = 0;

	// Scale
	QPointF scaleGrab;     //!< Screen offset of the grabbed point from the projected origin
	float scaleFactor = 1;
};

#endif // GLGIZMO_H

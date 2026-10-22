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

#include "glgizmo.h"

#include "gl/glscene.h"

#include <QLineF>
#include <QVector>

#include <cmath>


//! @file glgizmo.cpp TransformGizmo implementation

// Length of the gizmo arms in widget pixels
#define GIZMO_PIXELS 110.0f
// Pick tolerance in widget pixels
#define GIZMO_PICK 8.0

// Handle proportions relative to the arm length
#define SHAFT_START 0.12f
#define CONE_START 0.78f
#define CONE_RADIUS 0.065f
#define PLANE_MIN 0.25f
#define PLANE_MAX 0.5f
#define SCREEN_RING 1.2f

// Segments used to draw and pick the rotation rings
#define RING_SEGMENTS 72


static const Vector3 worldAxes[3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

static const float axisColors[3][3] = {
	{ 0.92f, 0.22f, 0.22f }, // X
	{ 0.36f, 0.86f, 0.24f }, // Y
	{ 0.24f, 0.46f, 1.00f }  // Z
};

static const float highlightColor[3] = { 1.0f, 0.85f, 0.1f };
static const float screenColor[3] = { 0.95f, 0.95f, 0.95f };


static double distToSegment( const QPointF & p, const QPointF & a, const QPointF & b )
{
	QPointF ab = b - a;
	double len2 = QPointF::dotProduct( ab, ab );
	double t = (len2 > 0) ? QPointF::dotProduct( p - a, ab ) / len2 : 0;
	t = qBound( 0.0, t, 1.0 );

	return QLineF( p, a + ab * t ).length();
}

static bool insideQuad( const QPointF & p, const QPointF * q )
{
	// Works for either winding, the quad is always convex
	int sign = 0;
	for ( int i = 0; i < 4; i++ ) {
		QPointF e = q[(i + 1) % 4] - q[i];
		QPointF w = p - q[i];
		double c = e.x() * w.y() - e.y() * w.x();
		int s = (c > 0) ? 1 : ((c < 0) ? -1 : 0);

		if ( s == 0 )
			continue;
		if ( sign == 0 )
			sign = s;
		else if ( s != sign )
			return false;
	}

	return true;
}

//! Rotates v, which must be perpendicular to the unit vector axis, by angle radians
static Vector3 rotatePerpendicular( const Vector3 & v, const Vector3 & axis, float angle )
{
	return v * std::cos( angle ) + Vector3::crossproduct( axis, v ) * std::sin( angle );
}

//! Sets the color of a handle, highlighting the hovered or dragged handle and fading the rest while dragging
static void handleColor( Scene * scene, TransformGizmo::Handle h, TransformGizmo::Handle hover, TransformGizmo::Handle active, const float * rgb, float alpha )
{
	if ( h == active || (active == TransformGizmo::HandleNone && h == hover) )
		rgb = highlightColor;
	else if ( active != TransformGizmo::HandleNone )
		alpha *= 0.3f;

	scene->setGLColor( rgb[0], rgb[1], rgb[2], alpha );
}

//! Draws a solid world aligned cube
static void drawSolidCube( Scene * scene, const Vector3 & c, float h )
{
	for ( int i = 0; i < 3; i++ ) {
		const Vector3 & n = worldAxes[i];
		const Vector3 & u = worldAxes[(i + 1) % 3];
		const Vector3 & v = worldAxes[(i + 2) % 3];

		for ( float side : { -1.0f, 1.0f } ) {
			Vector3 f = c + n * (h * side);
			Vector3 quad[4] = { f - u * h - v * h, f + u * h - v * h, f + u * h + v * h, f - u * h + v * h };
			scene->drawTriangles( quad, 4, nullptr, true, GL_TRIANGLE_FAN );
		}
	}
}

//! Multiplies the column-major 4x4 matrix m by the vector v
static void mul4( const double * m, const double * v, double * out )
{
	for ( int r = 0; r < 4; r++ )
		out[r] = m[r] * v[0] + m[4 + r] * v[1] + m[8 + r] * v[2] + m[12 + r] * v[3];
}

//! Inverts a 4x4 matrix, returns false if it is singular
static bool invert4( const double * m, double * out )
{
	double inv[16];

	inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
	inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
	inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
	inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
	inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
	inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
	inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
	inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
	inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
	inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
	inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
	inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
	inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
	inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
	inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
	inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

	double det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
	if ( det == 0 )
		return false;

	for ( int i = 0; i < 16; i++ )
		out[i] = inv[i] / det;

	return true;
}


/*
 * Camera
 */

void TransformGizmo::setCamera( const Transform & v, const Matrix4 & projection, const QSize & widgetSize )
{
	view = v;
	widget = QSize( qMax( widgetSize.width(), 1 ), qMax( widgetSize.height(), 1 ) );

	double p[16], mv[16];
	Matrix4 m = view.toMatrix4();
	for ( int i = 0; i < 16; i++ ) {
		p[i] = projection.data()[i];
		mv[i] = m.data()[i];
	}

	// A perspective projection has (0, 0, -1, 0) as its last row
	perspective = (p[15] == 0.0);

	for ( int c = 0; c < 4; c++ )
		mul4( p, mv + c * 4, viewProj + c * 4 );

	if ( !invert4( viewProj, invViewProj ) ) {
		for ( int i = 0; i < 16; i++ )
			invViewProj[i] = (i % 5 == 0) ? 1.0 : 0.0;
	}
}

void TransformGizmo::setOrigin( const Vector3 & o )
{
	origin = o;
}

bool TransformGizmo::project( const Vector3 & world, QPointF & out ) const
{
	if ( perspective && (view * world)[2] >= 0 )
		return false;

	double w[4] = { world[0], world[1], world[2], 1.0 };
	double clip[4];
	mul4( viewProj, w, clip );

	if ( std::fabs( clip[3] ) < 1e-12 )
		return false;

	// Normalized device coordinates to widget coordinates, widget Y points down
	double x = clip[0] / clip[3];
	double y = clip[1] / clip[3];
	out = QPointF( (x * 0.5 + 0.5) * widget.width(), (0.5 - y * 0.5) * widget.height() );
	return true;
}

void TransformGizmo::ray( const QPointF & pos, Vector3 & o, Vector3 & dir ) const
{
	double x = pos.x() / widget.width() * 2.0 - 1.0;
	double y = 1.0 - pos.y() / widget.height() * 2.0;

	double nearNdc[4] = { x, y, -1.0, 1.0 };
	double farNdc[4] = { x, y, 1.0, 1.0 };
	double n[4], f[4];
	mul4( invViewProj, nearNdc, n );
	mul4( invViewProj, farNdc, f );

	o = Vector3( n[0] / n[3], n[1] / n[3], n[2] / n[3] );
	Vector3 farPoint( f[0] / f[3], f[1] / f[3], f[2] / f[3] );
	dir = farPoint - o;
	dir.normalize();
}

Vector3 TransformGizmo::toViewer() const
{
	QPointF c;
	if ( !project( origin, c ) )
		return view.rotation.inverted() * Vector3( 0, 0, 1 );

	Vector3 ro, rd;
	ray( c, ro, rd );

	return -rd;
}

float TransformGizmo::scale() const
{
	// Keep the gizmo a constant size on screen by measuring
	// how many pixels one world unit covers at the gizmo depth
	Vector3 right = view.rotation.inverted() * Vector3( 1, 0, 0 );

	QPointF a, b;
	if ( !project( origin, a ) || !project( origin + right, b ) )
		return 1.0f;

	double ppu = QLineF( a, b ).length();
	if ( ppu < 1e-6 )
		return 1.0f;

	return GIZMO_PIXELS / ppu;
}

bool TransformGizmo::axisVisible( int axis, float len ) const
{
	QPointF a, b;
	if ( !project( origin, a ) || !project( origin + worldAxes[axis] * len, b ) )
		return false;

	return QLineF( a, b ).length() > GIZMO_PIXELS * 0.15;
}

bool TransformGizmo::planeVisible( int normal ) const
{
	return std::fabs( Vector3::dotproduct( worldAxes[normal], toViewer() ) ) > 0.2f;
}

Vector3 TransformGizmo::ringAxis( Handle handle ) const
{
	if ( handle == HandleScreen )
		return toViewer();

	return worldAxes[handle];
}

void TransformGizmo::ringBasis( Handle handle, Vector3 & u, Vector3 & v ) const
{
	if ( handle == HandleScreen ) {
		Vector3 a = toViewer();
		u = view.rotation.inverted() * Vector3( 1, 0, 0 );
		u = u - a * Vector3::dotproduct( u, a );
		u.normalize();
		v = Vector3::crossproduct( a, u );
		return;
	}

	// Cyclic order so that u x v is the axis
	u = worldAxes[(handle + 1) % 3];
	v = worldAxes[(handle + 2) % 3];
}

float TransformGizmo::ringRadius( Handle handle, float len ) const
{
	return (handle == HandleScreen) ? len * SCREEN_RING : len;
}

/*
 * Picking
 */

TransformGizmo::Handle TransformGizmo::hitTest( const QPointF & pos ) const
{
	switch ( mode ) {
	case Rotate:
		return hitTestRotate( pos );
	case Scale:
		return hitTestScale( pos );
	default:
		return hitTestTranslate( pos );
	}
}

TransformGizmo::Handle TransformGizmo::hitTestTranslate( const QPointF & pos ) const
{
	QPointF c;
	if ( !project( origin, c ) )
		return HandleNone;

	if ( QLineF( pos, c ).length() <= GIZMO_PICK + 1 )
		return HandleScreen;

	float len = scale();

	// Axes take priority over planes
	Handle best = HandleNone;
	double bestDist = GIZMO_PICK;
	for ( int i = 0; i < 3; i++ ) {
		if ( !axisVisible( i, len ) )
			continue;

		QPointF a, b;
		if ( !project( origin + worldAxes[i] * (len * SHAFT_START), a ) || !project( origin + worldAxes[i] * len, b ) )
			continue;

		double d = distToSegment( pos, a, b );
		if ( d <= bestDist ) {
			bestDist = d;
			best = Handle( HandleX + i );
		}
	}

	if ( best != HandleNone )
		return best;

	for ( int n = 0; n < 3; n++ ) {
		if ( !planeVisible( n ) )
			continue;

		const Vector3 & u = worldAxes[(n + 1) % 3];
		const Vector3 & v = worldAxes[(n + 2) % 3];

		Vector3 corners[4] = {
			origin + (u * PLANE_MIN + v * PLANE_MIN) * len,
			origin + (u * PLANE_MAX + v * PLANE_MIN) * len,
			origin + (u * PLANE_MAX + v * PLANE_MAX) * len,
			origin + (u * PLANE_MIN + v * PLANE_MAX) * len
		};

		QPointF q[4];
		bool ok = true;
		for ( int i = 0; i < 4; i++ )
			ok &= project( corners[i], q[i] );

		if ( ok && insideQuad( pos, q ) )
			return Handle( HandleYZ + n );
	}

	return HandleNone;
}

TransformGizmo::Handle TransformGizmo::hitTestRotate( const QPointF & pos ) const
{
	QPointF c;
	if ( !project( origin, c ) )
		return HandleNone;

	float len = scale();
	Vector3 tv = toViewer();

	Handle best = HandleNone;
	double bestDist = GIZMO_PICK;

	for ( Handle h : { HandleX, HandleY, HandleZ, HandleScreen } ) {
		Vector3 u, v;
		ringBasis( h, u, v );
		float r = ringRadius( h, len );

		// Only the half of an axis ring facing the camera can be picked
		QPointF prev;
		bool prevOk = false;
		for ( int s = 0; s <= RING_SEGMENTS; s++ ) {
			float f = 2 * PI * float( s ) / float( RING_SEGMENTS );
			Vector3 radial = u * std::cos( f ) + v * std::sin( f );
			bool front = (h == HandleScreen) || Vector3::dotproduct( radial, tv ) >= -0.05f;

			QPointF q;
			bool ok = front && project( origin + radial * r, q );

			if ( ok && prevOk ) {
				double d = distToSegment( pos, prev, q );
				if ( d < bestDist ) {
					bestDist = d;
					best = h;
				}
			}

			prev = q;
			prevOk = ok;
		}
	}

	return best;
}

TransformGizmo::Handle TransformGizmo::hitTestScale( const QPointF & pos ) const
{
	QPointF c;
	if ( !project( origin, c ) )
		return HandleNone;

	if ( QLineF( pos, c ).length() <= GIZMO_PICK + 3 )
		return HandleScreen;

	float len = scale();

	Handle best = HandleNone;
	double bestDist = GIZMO_PICK;
	for ( int i = 0; i < 3; i++ ) {
		if ( !axisVisible( i, len ) )
			continue;

		QPointF a, b;
		if ( !project( origin + worldAxes[i] * (len * SHAFT_START), a ) || !project( origin + worldAxes[i] * len, b ) )
			continue;

		double d = distToSegment( pos, a, b );
		if ( d <= bestDist ) {
			bestDist = d;
			best = Handle( HandleX + i );
		}
	}

	return best;
}


/*
 * Dragging
 */

bool TransformGizmo::beginDrag( Handle handle, const QPointF & pos )
{
	dragHandle = HandleNone;

	if ( handle == HandleNone )
		return false;

	dragOrigin = origin;
	dragPress = pos;

	bool ok;
	switch ( mode ) {
	case Rotate:
		ok = beginRotate( handle, pos );
		break;
	case Scale:
		ok = beginScale( handle, pos );
		break;
	default:
		ok = beginTranslate( handle, pos );
		break;
	}
	if ( ok )
		dragHandle = handle;

	return ok;
}

bool TransformGizmo::beginTranslate( Handle handle, const QPointF & pos )
{
	Vector3 ro, rd;
	ray( pos, ro, rd );

	lastDelta = Vector3();

	if ( handle <= HandleZ ) {
		const Vector3 & a = worldAxes[handle];
		Vector3 w0 = origin - ro;
		float b = Vector3::dotproduct( a, rd );
		float denom = 1.0f - b * b;

		if ( denom < 1e-4f )
			return false;

		dragParam = (b * Vector3::dotproduct( rd, w0 ) - Vector3::dotproduct( a, w0 )) / denom;
	} else {
		if ( handle == HandleScreen ) {
			dragNormal = -toViewer();
		} else {
			dragNormal = worldAxes[handle - HandleYZ];
		}

		float denom = Vector3::dotproduct( dragNormal, rd );
		if ( std::fabs( denom ) < 1e-4f )
			return false;

		float s = Vector3::dotproduct( dragNormal, origin - ro ) / denom;
		dragStart = ro + rd * s;
	}

	return true;
}

bool TransformGizmo::beginRotate( Handle handle, const QPointF & pos )
{
	if ( handle > HandleZ && handle != HandleScreen )
		return false;

	QPointF c;
	if ( !project( origin, c ) )
		return false;

	float len = scale();
	float r = ringRadius( handle, len );
	Vector3 tv = toViewer();
	Vector3 u, v;
	ringBasis( handle, u, v );

	dragAxis = ringAxis( handle );

	// The grabbed point is the ring sample closest to the mouse
	dragRadial = u;
	double best = 1e30;
	for ( int s = 0; s < RING_SEGMENTS; s++ ) {
		float f = 2 * PI * float( s ) / float( RING_SEGMENTS );
		Vector3 radial = u * std::cos( f ) + v * std::sin( f );

		if ( handle != HandleScreen && Vector3::dotproduct( radial, tv ) < -0.05f )
			continue;

		QPointF q;
		if ( !project( origin + radial * r, q ) )
			continue;

		double d = QLineF( pos, q ).length();
		if ( d < best ) {
			best = d;
			dragRadial = radial;
		}
	}

	// Seen at a reasonable angle: the mouse ray is intersected with the ring plane, which is exact.
	// Seen (almost) edge-on: the mouse drags linearly along the ring tangent at the grabbed point.
	float facing = Vector3::dotproduct( dragAxis, tv );
	dragPlanar = (handle == HandleScreen) || std::fabs( facing ) > 0.35f;
	lastPlaneAngle = 0;
	rawAngle = 0;
	angle = 0;

	if ( dragPlanar )
		return planeAngle( pos, lastPlaneAngle );

	const float step = 0.05f;
	bool ok = false;

	// The tangent at the grabbed point, unless it points into the screen.
	// Then roll the side of the ring facing the camera instead, which is always visible edge-on.
	sideDir = QPointF();
	for ( const Vector3 & radial : { dragRadial, tv } ) {
		Vector3 tangent = Vector3::crossproduct( dragAxis, radial );
		Vector3 grab = origin + radial * r;

		QPointF a, b;
		if ( !project( grab, a ) || !project( grab + tangent * (r * step), b ) )
			continue;

		QPointF d = b - a;
		double l = std::sqrt( QPointF::dotProduct( d, d ) );

		if ( l / step > 20.0 ) {
			tangentDir = d / l;
			pixelsPerRadian = l / step;
			ok = true;
			break;
		}

		// The grabbed point is the tip of the edge-on ring. Moving the mouse around the center
		// also rotates, as if the camera was tilted slightly towards the side it is on.
		QPointF tip( pos - c );
		double tipLen = std::sqrt( QPointF::dotProduct( tip, tip ) );
		if ( tipLen > 1.0 ) {
			// Counterclockwise on screen, widget Y points down
			sideDir = QPointF( tip.y(), -tip.x() ) / tipLen * ((facing >= 0) ? 1.0 : -1.0);
			sidePixelsPerRadian = tipLen;
		}
	}

	return ok;
}

bool TransformGizmo::planeAngle( const QPointF & pos, float & out ) const
{
	Vector3 ro, rd;
	ray( pos, ro, rd );

	float denom = Vector3::dotproduct( dragAxis, rd );
	if ( std::fabs( denom ) < 1e-4f )
		return false;

	float s = Vector3::dotproduct( dragAxis, dragOrigin - ro ) / denom;
	if ( perspective && s < 0 )
		return false;

	Vector3 w = (ro + rd * s) - dragOrigin;
	out = std::atan2( Vector3::dotproduct( w, Vector3::crossproduct( dragAxis, dragRadial ) ), Vector3::dotproduct( w, dragRadial ) );
	return true;
}

bool TransformGizmo::beginScale( Handle handle, const QPointF & pos )
{
	Q_UNUSED( handle );

	QPointF c;
	if ( !project( origin, c ) )
		return false;

	scaleGrab = pos - c;
	scaleFactor = 1;
	return true;
}

float TransformGizmo::dragScale( const QPointF & pos, float snapStep )
{
	if ( dragHandle == HandleNone || mode != Scale )
		return 1;

	QPointF c;
	project( origin, c );

	float f;
	double grab2 = QPointF::dotProduct( scaleGrab, scaleGrab );

	if ( dragHandle == HandleScreen || grab2 < 100.0 ) {
		// Grabbed at the center: right or up grows, left or down shrinks
		QPointF m = pos - dragPress;
		f = std::exp( (m.x() - m.y()) / 120.0 );
	} else {
		// Grabbed on an arm: the grabbed point follows the mouse, towards the center shrinks
		f = QPointF::dotProduct( pos - c, scaleGrab ) / grab2;
	}

	if ( snapStep > 0 )
		f = std::round( f / snapStep ) * snapStep;

	scaleFactor = qMax( f, (snapStep > 0) ? snapStep : 0.01f );
	return scaleFactor;
}

Vector3 TransformGizmo::dragDelta( const QPointF & pos )
{
	if ( dragHandle == HandleNone || mode != Translate )
		return Vector3();

	Vector3 ro, rd;
	ray( pos, ro, rd );

	if ( dragHandle <= HandleZ ) {
		const Vector3 & a = worldAxes[dragHandle];
		Vector3 w0 = dragOrigin - ro;
		float b = Vector3::dotproduct( a, rd );
		float d = Vector3::dotproduct( a, w0 );
		float e = Vector3::dotproduct( rd, w0 );
		float denom = 1.0f - b * b;

		// Mouse ray is parallel to the axis
		if ( denom < 1e-4f )
			return lastDelta;

		// Closest point would be behind the camera
		if ( perspective && (e - b * d) / denom < 0 )
			return lastDelta;

		float t = (b * e - d) / denom;
		lastDelta = a * (t - dragParam);
	} else {
		float denom = Vector3::dotproduct( dragNormal, rd );
		if ( std::fabs( denom ) < 1e-4f )
			return lastDelta;

		float s = Vector3::dotproduct( dragNormal, dragOrigin - ro ) / denom;
		if ( perspective && s < 0 )
			return lastDelta;

		lastDelta = (ro + rd * s) - dragStart;
	}

	return lastDelta;
}

Matrix TransformGizmo::dragRotation( const QPointF & pos, float snapDegrees )
{
	if ( dragHandle == HandleNone || mode != Rotate )
		return Matrix();

	if ( dragPlanar ) {
		// Accumulate so that multiple turns around the center work
		float a;
		if ( planeAngle( pos, a ) ) {
			float d = a - lastPlaneAngle;
			if ( d > PI )
				d -= 2 * PI;
			else if ( d < -PI )
				d += 2 * PI;

			rawAngle += d;
			lastPlaneAngle = a;
		}
	} else {
		QPointF m = pos - dragPress;
		rawAngle = QPointF::dotProduct( m, tangentDir ) / pixelsPerRadian;

		if ( !sideDir.isNull() )
			rawAngle += QPointF::dotProduct( m, sideDir ) / sidePixelsPerRadian;
	}

	angle = rawAngle;
	if ( snapDegrees > 0 ) {
		float s = snapDegrees / 180.0f * PI;
		angle = std::round( angle / s ) * s;
	}

	Quat q;
	q.fromAxisAngle( dragAxis, angle );

	Matrix m;
	m.fromQuat( q );
	return m;
}


/*
 * Drawing
 */

void TransformGizmo::draw( Scene * scene, Handle hover, Handle active, float pixelScale ) const
{
	QPointF c;
	if ( !project( origin, c ) )
		return;

	// Draw on top of everything, in world space
	scene->loadModelViewMatrix( view );
	glDisable( GL_DEPTH_TEST );
	glDepthMask( GL_FALSE );
	glDisable( GL_CULL_FACE );

	switch ( mode ) {
	case Rotate:
		drawRotate( scene, hover, active, pixelScale );
		break;
	case Scale:
		drawScale( scene, hover, active, pixelScale );
		break;
	default:
		drawTranslate( scene, hover, active, pixelScale );
		break;
	}

	glDepthMask( GL_TRUE );
	glEnable( GL_DEPTH_TEST );
}

void TransformGizmo::drawTranslate( Scene * scene, Handle hover, Handle active, float ps ) const
{
	float len = scale();

	// Guide line through the drag origin while dragging an axis
	if ( active >= HandleX && active <= HandleZ ) {
		const Vector3 & a = worldAxes[active];
		scene->setGLLineWidth( 1.0f * ps );
		scene->setGLColor( axisColors[active][0], axisColors[active][1], axisColors[active][2], 0.6f );
		scene->drawLine( dragOrigin - a * (len * 100.0f), dragOrigin + a * (len * 100.0f) );
	}

	// Planes
	for ( int n = 0; n < 3; n++ ) {
		if ( !planeVisible( n ) )
			continue;

		Handle h = Handle( HandleYZ + n );
		const Vector3 & u = worldAxes[(n + 1) % 3];
		const Vector3 & v = worldAxes[(n + 2) % 3];

		Vector3 corners[4] = {
			origin + (u * PLANE_MIN + v * PLANE_MIN) * len,
			origin + (u * PLANE_MAX + v * PLANE_MIN) * len,
			origin + (u * PLANE_MAX + v * PLANE_MAX) * len,
			origin + (u * PLANE_MIN + v * PLANE_MAX) * len
		};

		handleColor( scene, h, hover, active, axisColors[n], 0.3f );
		scene->drawTriangles( corners, 4, nullptr, true, GL_TRIANGLE_FAN );

		handleColor( scene, h, hover, active, axisColors[n], 0.9f );
		scene->setGLLineWidth( 1.5f * ps );
		scene->drawLines( corners, 4, nullptr, GL_LINE_LOOP );
	}

	// Axes
	const int sd = 16;
	for ( int i = 0; i < 3; i++ ) {
		if ( !axisVisible( i, len ) )
			continue;

		Handle h = Handle( HandleX + i );
		const Vector3 & a = worldAxes[i];
		const Vector3 & u = worldAxes[(i + 1) % 3];
		const Vector3 & v = worldAxes[(i + 2) % 3];

		handleColor( scene, h, hover, active, axisColors[i], 1.0f );

		scene->setGLLineWidth( 3.0f * ps );
		scene->drawLine( origin + a * (len * SHAFT_START), origin + a * (len * CONE_START) );

		Vector3 tip = origin + a * len;
		Vector3 base = origin + a * (len * CONE_START);
		float r = len * CONE_RADIUS;

		Vector3 cone[sd + 2], cap[sd + 2];
		cone[0] = tip;
		cap[0] = base;
		for ( int s = 0; s <= sd; s++ ) {
			float f = float( 2 * PI * s / sd );
			cone[s + 1] = cap[s + 1] = base + u * (r * std::cos( f )) + v * (r * std::sin( f ));
		}
		scene->drawTriangles( cone, sd + 2, nullptr, true, GL_TRIANGLE_FAN );
		scene->drawTriangles( cap, sd + 2, nullptr, true, GL_TRIANGLE_FAN );
	}

	// View plane handle
	handleColor( scene, HandleScreen, hover, active, screenColor, 1.0f );
	scene->setGLPointSize( 10.0f * ps );
	scene->drawPoints( &origin, 1 );
}

void TransformGizmo::drawRotate( Scene * scene, Handle hover, Handle active, float ps ) const
{
	float len = scale();
	Vector3 tv = toViewer();
	Vector3 ring[RING_SEGMENTS];

	// Silhouette circle, for orientation only
	{
		Vector3 u, v;
		ringBasis( HandleScreen, u, v );
		for ( int s = 0; s < RING_SEGMENTS; s++ ) {
			float f = float( 2 * PI * s / RING_SEGMENTS );
			ring[s] = origin + (u * std::cos( f ) + v * std::sin( f )) * len;
		}
		scene->setGLLineWidth( 1.0f * ps );
		scene->setGLColor( 0.6f, 0.6f, 0.6f, (active == HandleNone) ? 0.5f : 0.2f );
		scene->drawLines( ring, RING_SEGMENTS, nullptr, GL_LINE_LOOP );
	}

	// Swept angle while dragging
	if ( active != HandleNone && dragHandle == active ) {
		const float * rgb = (active == HandleScreen) ? screenColor : axisColors[active];
		float r = ringRadius( active, len );
		int steps = qMax( 2, int( std::fabs( angle ) / (2 * PI) * RING_SEGMENTS ) + 1 );

		QVector<Vector3> fan( steps + 2 );
		fan[0] = origin;
		for ( int s = 0; s <= steps; s++ )
			fan[s + 1] = origin + rotatePerpendicular( dragRadial, dragAxis, angle * s / steps ) * r;
		scene->setGLColor( rgb[0], rgb[1], rgb[2], 0.25f );
		scene->drawTriangles( fan.constData(), size_t( fan.size() ), nullptr, true, GL_TRIANGLE_FAN );

		Vector3 spokes[4] = {
			origin, origin + dragRadial * r,
			origin, origin + rotatePerpendicular( dragRadial, dragAxis, angle ) * r
		};
		scene->setGLLineWidth( 1.5f * ps );
		scene->setGLColor( rgb[0], rgb[1], rgb[2], 0.8f );
		scene->drawLines( spokes, 4 );
	}

	// Axis rings, the half facing away from the camera is faded
	for ( Handle h : { HandleX, HandleY, HandleZ } ) {
		Vector3 u, v;
		ringBasis( h, u, v );

		for ( int pass = 0; pass < 2; pass++ ) {
			bool front = (pass == 1);
			QVector<Vector3> segments;
			segments.reserve( RING_SEGMENTS * 2 );

			for ( int s = 0; s < RING_SEGMENTS; s++ ) {
				float f0 = float( 2 * PI * s / RING_SEGMENTS );
				float f1 = float( 2 * PI * (s + 1) / RING_SEGMENTS );
				Vector3 r0 = u * std::cos( f0 ) + v * std::sin( f0 );
				Vector3 r1 = u * std::cos( f1 ) + v * std::sin( f1 );

				bool isFront = Vector3::dotproduct( r0 + r1, tv ) >= -0.1f;
				if ( isFront != front )
					continue;

				segments.append( origin + r0 * len );
				segments.append( origin + r1 * len );
			}

			if ( segments.isEmpty() )
				continue;

			scene->setGLLineWidth( (front ? 3.0f : 1.5f) * ps );
			handleColor( scene, h, hover, active, axisColors[h], front ? 1.0f : 0.25f );
			scene->drawLines( segments.constData(), size_t( segments.size() ) );
		}
	}

	// View axis ring
	{
		Vector3 u, v;
		ringBasis( HandleScreen, u, v );
		float r = ringRadius( HandleScreen, len );
		for ( int s = 0; s < RING_SEGMENTS; s++ ) {
			float f = float( 2 * PI * s / RING_SEGMENTS );
			ring[s] = origin + (u * std::cos( f ) + v * std::sin( f )) * r;
		}
		scene->setGLLineWidth( 2.0f * ps );
		handleColor( scene, HandleScreen, hover, active, screenColor, 0.9f );
		scene->drawLines( ring, RING_SEGMENTS, nullptr, GL_LINE_LOOP );
	}

	// Pivot
	scene->setGLColor( screenColor[0], screenColor[1], screenColor[2], 0.9f );
	scene->setGLPointSize( 6.0f * ps );
	scene->drawPoints( &origin, 1 );
}

void TransformGizmo::drawScale( Scene * scene, Handle hover, Handle active, float ps ) const
{
	float len = scale();
	bool dragging = (active != HandleNone && dragHandle == active);

	// The scale is uniform, so all handles highlight together
	bool lit = (hover != HandleNone || active != HandleNone);
	float f = dragging ? qBound( 0.25f, scaleFactor, 4.0f ) : 1.0f;

	for ( int i = 0; i < 3; i++ ) {
		if ( !axisVisible( i, len ) )
			continue;

		const Vector3 & a = worldAxes[i];
		const float * rgb = lit ? highlightColor : axisColors[i];

		// The original size while dragging
		if ( dragging ) {
			scene->setGLLineWidth( 1.0f * ps );
			scene->setGLColor( axisColors[i][0], axisColors[i][1], axisColors[i][2], 0.4f );
			scene->drawLine( origin + a * (len * SHAFT_START), origin + a * len );
		}

		float tip = len * f;
		float cube = len * CONE_RADIUS;

		scene->setGLColor( rgb[0], rgb[1], rgb[2], 1.0f );
		scene->setGLLineWidth( 3.0f * ps );
		scene->drawLine( origin + a * (len * SHAFT_START), origin + a * (tip - cube) );

		drawSolidCube( scene, origin + a * (tip - cube), cube );
	}

	const float * rgb = lit ? highlightColor : screenColor;
	scene->setGLColor( rgb[0], rgb[1], rgb[2], 0.9f );
	drawSolidCube( scene, origin, len * 0.06f );
}

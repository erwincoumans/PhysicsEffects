/*
   Copyright (C) 2009 Sony Computer Entertainment Inc.
   All rights reserved.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#include <vectormath_aos.h>

#include "Capsule.h"
#include "Sphere.h"
#include "vec_utils.h"

using namespace Vectormath::Aos;

inline
void
segmentPointClosestPoints(
	Vector3& ptsVector,
	Vector3& offsetA,
	float& tA,
	Vector3 translation,
	Vector3 dirA, float hLenA )
{
	// compute the parameters of the closest points on each line segment

	tA = dot(dirA,translation);

	if ( tA < -hLenA )
		tA = -hLenA;
	else if ( tA > hLenA )
		tA = hLenA;

	// compute the closest point on segment relative to its center.

	offsetA = dirA * tA;
	ptsVector = translation - offsetA;
}

inline
void
segmentPointNormal(     Vector3& normal, Vector3 ptsVector )
{
	// compute the unit direction vector between the closest points.
	// with convex objects, you want the unit direction providing the largest gap between the
	// objects when they're projected onto it.  So, if you have a few candidates covering different
	// configurations of the objects, you can compute them all, test the gaps and pick best axis
	// based on this.  Some directions might be degenerate, and the normalized() function tests for
	// degeneracy and returns an arbitrary unit vector in that case.

	// closest points vector

	normal = safeNormalize(ptsVector);
}

float
capsuleSphereDistance(
	Vector3 & normal,
	CapsulePoint & capsulePointA,
	SpherePoint & spherePointB,
	const Capsule & capsuleA, const Transform3 & transformA,
	Sphere sphereB, const Transform3 & transformB,
	float distanceThreshold )
{
	Vector3 directionA = transformA.getUpper3x3().getCol0();
	Vector3 translationA = transformA.getTranslation();
	Vector3 translationB = transformB.getTranslation();

	// translation between centers of capsule and sphere

	Vector3 translation = translationB - translationA;

	// compute the closest point on the capsule line segment to the sphere center

	Vector3 ptsVector;
	Vector3 offsetA;
	float tA;

	segmentPointClosestPoints( ptsVector, offsetA, tA, translation, directionA, capsuleA.hLength );

	float distance = length(ptsVector) - capsuleA.radius - sphereB.radius;

	if ( distance > distanceThreshold )
		return distance;

	// compute the contact normal

	segmentPointNormal( normal, ptsVector );

	// compute points on capsule and sphere

	capsulePointA.lineParam = tA;
	capsulePointA.localPoint = Point3( transpose(transformA.getUpper3x3()) * ( offsetA + normal * capsuleA.radius ) );
	capsulePointA.setEdgeFeature();

	spherePointB.localPoint = Point3( transpose(transformB.getUpper3x3()) * ( -normal * sphereB.radius ) );
	spherePointB.setVertexFeature();

	// return distance

	return distance;
}

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
#include "vec_utils.h"

using namespace Vectormath::Aos;

inline
void
segmentsClosestPoints(
	Vector3& ptsVector,
	Vector3& offsetA,
	Vector3& offsetB,
	float& tA, float& tB,
	Vector3 translation,
	Vector3 dirA, float hlenA,
	Vector3 dirB, float hlenB )
{
	// compute the parameters of the closest points on each line segment

	float dirA_dot_dirB = dot(dirA,dirB);
	float dirA_dot_trans = dot(dirA,translation);
	float dirB_dot_trans = dot(dirB,translation);

	float denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

	if ( denom == 0.0f ) {
		tA = 0.0f;
	} else {
		tA = ( dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB ) / denom;
		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	}

	tB = tA * dirA_dot_dirB - dirB_dot_trans;

	if ( tB < -hlenB ) {
		tB = -hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	} else if ( tB > hlenB ) {
		tB = hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	}

	// compute the closest points relative to segment centers.

	offsetA = dirA * tA;
	offsetB = dirB * tB;

	ptsVector = translation - offsetA + offsetB;
}

inline
void
segmentsNormal( Vector3& normal, Vector3 ptsVector )
{
	// compute the unit direction vector between the closest points.
	// with convex objects, you want the unit direction providing the largest gap between the
	// objects when they're projected onto it.  So, if you have a few candidates covering different
	// configurations of the objects, you can compute them all, test the gaps and pick best axis
	// based on this.  Some directions might be degenerate, and the normalized() function tests for
	// degeneracy and returns an arbitrary unit vector in that case.

	Vector3 testDir;

	// closest points vector

	normal = safeNormalize(ptsVector);
}

float
capsuleCapsuleDistance(
	Vector3 & normal,
	CapsulePoint & capsulePointA,
	CapsulePoint & capsulePointB,
	const Capsule & capsuleA, const Transform3 & transformA,
	const Capsule & capsuleB, const Transform3 & transformB,
	float distanceThreshold )
{
	Vector3 directionA = transformA.getUpper3x3().getCol0();
	Vector3 translationA = transformA.getTranslation();
	Vector3 directionB = transformB.getUpper3x3().getCol0();
	Vector3 translationB = transformB.getTranslation();

	// translation between centers

	Vector3 translation = translationB - translationA;

	// compute the closest points of the capsule line segments

	Vector3 ptsVector;           // the vector between the closest points
	Vector3 offsetA, offsetB;    // offsets from segment centers to their closest points
	float tA, tB;              // parameters on line segment

	segmentsClosestPoints( ptsVector, offsetA, offsetB, tA, tB, translation,
						   directionA, capsuleA.hLength, directionB, capsuleB.hLength );

	float distance = length(ptsVector) - capsuleA.radius - capsuleB.radius;

	if ( distance > distanceThreshold )
		return distance;

	// compute the contact normal

	segmentsNormal( normal, ptsVector );

	// compute points on capsules

	capsulePointA.lineParam = tA;
	capsulePointB.lineParam = tB;

	capsulePointA.localPoint = Point3( transpose(transformA.getUpper3x3()) * ( offsetA + normal * capsuleA.radius ) );
	capsulePointB.localPoint = Point3( transpose(transformB.getUpper3x3()) * ( offsetB - normal * capsuleB.radius ) );

	capsulePointA.setEdgeFeature();
	capsulePointB.setEdgeFeature();

	// return distance

	return distance;
}


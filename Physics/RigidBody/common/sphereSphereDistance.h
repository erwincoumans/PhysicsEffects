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

#ifndef __SPHERESPHREDISTANCE_H__
#define __SPHERESPHREDISTANCE_H__

#include "Sphere.h"

//---------------------------------------------------------------------------
// sphereSphereDistance:
//
// returns:
//    positive or negative distance between two spheres.
//
// args:
//    Vector3& normal: set to a unit contact normal pointing from sphere A to sphere B.
//
//    SpherePoint& spherePointA, SpherePoint& spherePointB:
//       set to a closest point or point of penetration on each object
//
//    const Sphere& sphereA, const Sphere& sphereB: sphere sizes
//
//    const Transform3 & transformA,B: the translation gives the center position
//
//    float distanceThreshold:
//       the algorithm will exit early if it finds that the objects are more distant than this
//       threshold, and not compute a contact normal or points.  if this distance returned
//       exceeds the threshold, all the other output data may not have been computed.  by
//       default, this is set to MAX_FLOAT so it will have no effect.
//
//---------------------------------------------------------------------------

#define lenSqrTol 1.0e-30f

inline float
sphereSphereDistance(
	Vector3 & direction,
	SpherePoint & spherePointA,
	SpherePoint & spherePointB,
	Sphere sphereA, const Transform3 & transformA,
	Sphere sphereB, const Transform3 & transformB,
	float distanceThreshold = FLT_MAX )
{
	Vector3 translationA = transformA.getTranslation();
	Vector3 translationB = transformB.getTranslation();

	// get the offset vector between sphere centers

	Vector3 offsetAB;

	offsetAB = translationB - translationA;

	// normalize the offset to compute the direction vector

	float distSqr = dot(offsetAB,offsetAB);
	float dist = sqrtf(distSqr);
	float sphereDist = dist - sphereA.radius - sphereB.radius;

	if ( sphereDist > distanceThreshold ) {
		return sphereDist;
	}

	if ( distSqr > lenSqrTol ) {
		float distInv = 1.0f / dist;

		direction = offsetAB * distInv;
	} else {
		direction = Vector3(0.0f, 0.0f, 1.0f);
	}

	// compute the points on the spheres, in world space

	spherePointA.localPoint = Point3( transpose(transformA.getUpper3x3()) * ( direction * sphereA.radius ) );
	spherePointB.localPoint = Point3( transpose(transformB.getUpper3x3()) * ( -direction * sphereB.radius ) );

	spherePointA.setVertexFeature();
	spherePointB.setVertexFeature();

	// return the distance between the spheres

	return sphereDist;
}

#endif /* __SPHERESPHREDISTANCE_H__ */

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

#include "Box.h"
#include "Sphere.h"
#include "Physics/Base/PhysicsCommon.h"

using namespace Vectormath::Aos;

//---------------------------------------------------------------------------
// lenSqrTol: minimum square of length for safe normalize.
//---------------------------------------------------------------------------

static const float lenSqrTol = 1.0e-30f;

//---------------------------------------------------------------------------
// separating axis tests: gaps along each axis are computed, and the axis with the maximum
// gap is stored.  cross product axes must be normalized.
//---------------------------------------------------------------------------

#define AaxisTest( dim, letter, first )                                                         \
{                                                                                               \
   float gap = gapsA.get##letter();                                                             \
                                                                                                \
   if ( gap > distanceThreshold )                                                               \
   {                                                                                            \
      return gap;                                                                               \
   }                                                                                            \
                                                                                                \
   if ( first )                                                                                 \
   {                                                                                            \
      maxGap = gap;                                                                             \
      faceDimA = dim;                                                                           \
      axisA = mulPerElem( identity.getCol##dim(), signsA );                                     \
   }                                                                                            \
   else                                                                                         \
   {                                                                                            \
      if ( gap > maxGap )                                                                       \
      {                                                                                         \
         maxGap = gap;                                                                          \
         faceDimA = dim;                                                                        \
         axisA = mulPerElem( identity.getCol##dim(), signsA );                                  \
      }                                                                                         \
   }                                                                                            \
}

inline
float
VertexBFaceATest(
	Vector3& ptsVec,
	float& t0,
	float& t1,
	const Vector3& hA,
	Vector3 offsetAB )
{
	// compute center of sphere in box's coordinate system

	Vector3 cptsVec = Vector3(offsetAB);

	// compute the parameters of the point on the face

	t0 = cptsVec[0];
	t1 = cptsVec[1];

	if ( t0 > hA[0] )
		t0 = hA[0];
	else if ( t0 < -hA[0] )
		t0 = -hA[0];
	if ( t1 > hA[1] )
		t1 = hA[1];
	else if ( t1 < -hA[1] )
		t1 = -hA[1];

	cptsVec[0] -= t0;
	cptsVec[1] -= t1;

	ptsVec = Vector3( cptsVec );

	return dot(ptsVec,ptsVec);
}

float
boxSphereDistance(
	Vector3& normal,
	BoxPoint& boxPointA,
	SpherePoint& spherePointB,
	Box boxA, const Transform3& transformA,
	const Sphere& sphereB, const Transform3 & transformB,
	float distanceThreshold )
{
	Matrix3 identity = Matrix3::identity();
	Vector3 ident[3];
	ident[0] = identity.getCol0();
	ident[1] = identity.getCol1();
	ident[2] = identity.getCol2();

	// offsetAB is vector from A's center to B's center, in A's coordinate system

	Vector3 translationB = transformB.getTranslation();
	Vector3 offsetAB = transpose(transformA.getUpper3x3()) * ( translationB -
					   transformA.getTranslation() );

	// find separating axis with largest gap between objects

	Vector3 axisA;
	int   faceDimA;
	float maxGap;

	Vector3 gapsA = absPerElem(offsetAB) - boxA.half - Vector3(sphereB.radius);
	Vector3 signsA = copySignPerElem(Vector3(1.0f),offsetAB);

	AaxisTest( 0, X, true );
	AaxisTest( 1, Y, false );
	AaxisTest( 2, Z, false );

	// choose face in this direction, and make a new coordinate system which the z axis = face
	// normal, x and y axes tangent to the face.  to transform vectors into this coordinate
	// system, will use a permutation matrix.

	int dimA[3];

	dimA[2] = faceDimA;
	dimA[0] = (faceDimA+1)%3;
	dimA[1] = (faceDimA+2)%3;

	Matrix3 apermCol;

	apermCol.setCol0(ident[dimA[0]]);
	apermCol.setCol1(ident[dimA[1]]);
	apermCol.setCol2(ident[dimA[2]]);

	Matrix3 apermRow = transpose(apermCol);

	// permute vectors

	Vector3 halfA_perm = apermRow * boxA.half;
	Vector3 offsetAB_perm = apermRow * offsetAB;
	Vector3 signsA_perm = apermRow * signsA;

	// compute the vector between the center of the box face and the sphere center

	float signA2 = signsA_perm.getZ();
	float scaleA2 = halfA_perm.getZ() * signA2;
	offsetAB_perm.setZ( offsetAB_perm.getZ() - scaleA2 );

	// find point on face closest to sphere center

	float t0, t1;
	float minDistSqr;
	Vector3 closestPtsVec_perm;
	Point3 localPointA_perm;

	minDistSqr = VertexBFaceATest( closestPtsVec_perm, t0, t1, Vector3( halfA_perm ), offsetAB_perm );

	localPointA_perm = Point3( t0, t1, scaleA2 );

	// compute normal

	bool centerInside = ( signA2 * closestPtsVec_perm.getZ() < 0.0f );

	if ( centerInside || ( minDistSqr < lenSqrTol ) ) {
		normal = transformA * axisA;
	} else {
		Vector3 closestPtsVec = apermCol * closestPtsVec_perm;
		normal = transformA * ( closestPtsVec * ( 1.0f / sqrtf( minDistSqr ) ) );
	}

	// compute box point

	boxPointA.localPoint = Point3( apermCol * Vector3( localPointA_perm ) );
	boxPointA.setFaceFeature( faceDimA, localPointA_perm.getZ() > 0.0f );

	// compute sphere point

	spherePointB.localPoint = Point3( transpose(transformB.getUpper3x3()) * ( -normal * sphereB.radius ) );
	spherePointB.setVertexFeature();

	// return distance

	if ( centerInside ) {
		return -sqrtf( minDistSqr ) - sphereB.radius;
	} else {
		return sqrtf( minDistSqr ) - sphereB.radius;
	}
}


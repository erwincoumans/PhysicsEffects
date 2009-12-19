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

#include <float.h>
#include <vectormath_aos.h>

#include "Physics/RigidBody/common/Contact.h"
#include "Physics/RigidBody/common/CollObject.h"

#include "convexSphereDistance.h"
#include "convexBoxDistance.h"
#include "convexCapsuleDistance.h"
#include "convexConvexDistance.h"
#include "contactHeightField.h"
#include "HeightFieldFunction.h"

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/Base/PerfCounter.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"
#include "Physics/RigidBody/common/SubData.h"


using namespace Vectormath::Aos;

#define SET_CONTACT_POINT(contactPoint,dist,nml,pntA,trnsA,primA,pntB,trnsB,primB) \
	contactPoint.distance = dist;\
	contactPoint.setNormal(nml);\
	contactPoint.setA(pntA, trnsA, primA);\
	contactPoint.setB(pntB, trnsB, primB);\
	contactPoint.subData.type = SubData::SubDataNone;

class CachedConvexMesh {
public:
	ConvexMesh *mesh;

	CachedConvexMesh(const CollPrim &prim)
	{
		mesh = prim.getConvexMesh();
	}

	~CachedConvexMesh()
	{
	}
	
	ConvexMesh *getMesh()
	{
		return mesh;
	}
};

///////////////////////////////////////////////////////////////////////////////

int primContactsSphereConvex(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA,pointB;
	
	CachedConvexMesh meshB(primB);

	float distance = closestConvexSphere( testNormal, pointB, pointA,
								    meshB.getMesh(), primTransformB,
								    primA.getSphere(), primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,-testNormal,
			pointA, relTransformA, primIndexA,
			pointB, relTransformB, primIndexB);
		return 1;
	}
	
	return 0;
}

int primContactsConvexSphere(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA,pointB;

	CachedConvexMesh meshA(primA);

	float distance = closestConvexSphere( testNormal, pointA, pointB,
									meshA.getMesh(), primTransformA,
									primB.getSphere(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,testNormal,
			pointA, relTransformA, primIndexA,
			pointB, relTransformB, primIndexB);
		return 1;
	}
	
	return 0;
}

int primContactsBoxConvex(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA,pointB;

	CachedConvexMesh meshB(primB);

	float distance = closestConvexBox( testNormal, pointB, pointA,
							   meshB.getMesh(), primTransformB,
							   primA.getBox(), primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,-testNormal,
			pointA, relTransformA, primIndexA,
			pointB, relTransformB, primIndexB);
		return 1;
	}
	
	return 0;
}

int primContactsConvexBox(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA,pointB;

	CachedConvexMesh meshA(primA);

	float distance = closestConvexBox( testNormal, pointA, pointB,
							   meshA.getMesh(), primTransformA,
							   primB.getBox(), primTransformB, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,testNormal,
			pointA, relTransformA, primIndexA,
			pointB, relTransformB, primIndexB);
		return 1;
	}
	
	return 0;
}

int primContactsCapsuleConvex(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA,pointB;

	CachedConvexMesh meshB(primB);

	float distance = closestConvexCapsule( testNormal, pointB, pointA,
							   meshB.getMesh(), primTransformB,
							   primA.getCapsule(), primTransformA, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,-testNormal,
			pointA, relTransformA, primIndexA,
			pointB, relTransformB, primIndexB);
		return 1;
	}
	
	return 0;
}

int primContactsConvexCapsule(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA,pointB;

	CachedConvexMesh meshA(primA);

	float distance = closestConvexCapsule( testNormal, pointA, pointB,
							   meshA.getMesh(), primTransformA,
							   primB.getCapsule(), primTransformB, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,testNormal,
			pointA, relTransformA, primIndexA,
			pointB, relTransformB, primIndexB);
		return 1;
	}
	
	return 0;
}

int primContactsConvexHeightField(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{
	(void) primTransformB;
	(void) objsInContactDist;

	bool ret =false;
	
	HeightField *heightfield;

	CachedConvexMesh meshA(primA);

	heightfield = primB.getHeightField();

	Transform3 transformBA = orthoInverse(primTransformB) * primTransformA;

	for(uint32_t i=0;i<meshA.getMesh()->numVerts;i++) {
		Point3 cpB(0.0f);
		Vector3 nml(0.0f);
		float dist = 0.0f;

		if (contactHeightField(
			heightfield,
			transformBA *Point3(meshA.getMesh()->verts[i]),
			cpB, nml, dist) && dist < cp[0].distance) {

			Point3 cpA(meshA.getMesh()->verts[i]);

			SET_CONTACT_POINT(cp[0],dist,primTransformB.getUpper3x3()*nml,
				cpA,relTransformA,primIndexA,
				cpB,relTransformB,primIndexB);
			ret = true;
		}
	}


	return ret?1:0;
}

int primContactsHeightFieldConvex(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{
	(void) primTransformA;
	(void) objsInContactDist;

	bool ret =false;

	HeightField *heightfield;

	CachedConvexMesh meshB(primB);

	heightfield = primA.getHeightField();

	Transform3 transformAB = orthoInverse(primTransformA) * primTransformB;

	for(uint32_t i=0;i<meshB.getMesh()->numVerts;i++) {
		Point3 cpA(0.0f);
		Vector3 nml(0.0f);
		float dist = 0.0f;

		if(contactHeightField(
			heightfield,
			transformAB *Point3(meshB.getMesh()->verts[i]),
			cpA, nml, dist) && dist < cp[0].distance) {

			Point3 cpB(meshB.getMesh()->verts[i]);

			SET_CONTACT_POINT(cp[0],dist,-primTransformA.getUpper3x3()*nml,
				cpA,relTransformA,primIndexA,
				cpB,relTransformB,primIndexB);
			ret = true;
		}
	}


	return ret?1:0;
}

int primContactsConvexConvex(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA,pointB;

	CachedConvexMesh meshA(primA);
	CachedConvexMesh meshB(primB);

	float distance = closestConvexConvex( testNormal, pointA, pointB, 
								   meshA.getMesh(), primTransformA,
								   meshB.getMesh(), primTransformB, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,testNormal,
			pointA,relTransformA,primIndexA,
			pointB,relTransformB,primIndexB);
		return 1;
	}

	return 0;
}

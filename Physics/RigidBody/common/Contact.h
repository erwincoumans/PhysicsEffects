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

#ifndef __CONTACT_H__
#define __CONTACT_H__

#include <float.h>
#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/Base/SortCommon.h"
#include "Physics/Base/SimdFunc.h"
#include "Physics/RigidBody/common/CollObject.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"
#include "Physics/RigidBody/common/SubData.h"
#include "Physics/RigidBody/common/ConstraintCache.h"

#define NO_CONTACT_DISTANCE	999.0f
#define CONTACT_EPSILON		0.0f
#define SWAP(type, x, y) do {type t; t=x; x=y; y=t; } while (0)

///////////////////////////////////////////////////////////////////////////////
// Contact Point

struct ContactPoint
{
	ConstraintCache constraints[3];

	uint8_t duration;
	uint8_t primIdxA;
	uint8_t primIdxB;
	SubData subData;

	float distance;
	float localVelocityA[3];
	float localVelocityB[3];
	float localPointA[3];
	float localPointB[3];

	// --------------------------------------------

	ContactPoint()
	{
		reset();
	}

	void reset()
	{
		distance = FLT_MAX;
		duration = 0;
		subData.type = 0;
		localVelocityA[0] = 0.0f;
		localVelocityA[1] = 0.0f;
		localVelocityA[2] = 0.0f;
		localVelocityB[0] = 0.0f;
		localVelocityB[1] = 0.0f;
		localVelocityB[2] = 0.0f;
		constraints[0].normal[0] = 0.0f;
		constraints[0].normal[1] = 0.0f;
		constraints[0].normal[2] = 0.0f;
		constraints[0].accumImpulse = 0.0f;
		constraints[1].accumImpulse = 0.0f;
		constraints[2].accumImpulse = 0.0f;
	}
	
	void exchange()
	{
		SWAP(float,localPointA[0],localPointB[0]);
		SWAP(float,localPointA[1],localPointB[1]);
		SWAP(float,localPointA[2],localPointB[2]);
		SWAP(uint8_t,primIdxA,primIdxB);
		SWAP(float,localVelocityA[0],localVelocityB[0]);
		SWAP(float,localVelocityA[1],localVelocityB[1]);
		SWAP(float,localVelocityA[2],localVelocityB[2]);
		setNormal(-getNormal());
	}

	void setA(Point3 &contactPoint, const Transform3 & objectRelativeTransform,uint8_t primIdx_)
	{
		Vector3 p(objectRelativeTransform * contactPoint);
		setLocalPointA(p);
		primIdxA = primIdx_;
	}

	void setB(Point3 &contactPoint, const Transform3 & objectRelativeTransform,uint8_t primIdx_)
	{
		Vector3 p(objectRelativeTransform * contactPoint);
		setLocalPointB(p);
		primIdxB = primIdx_;
	}
	
	void setLocalPointA(const Vector3 &p) {store_Vector3(p,localPointA);}
	void setLocalPointB(const Vector3 &p) {store_Vector3(p,localPointB);}
	void setLocalVelocityA(const Vector3 &v) {store_Vector3(v,localVelocityA);}
	void setLocalVelocityB(const Vector3 &v) {store_Vector3(v,localVelocityB);}
	void setNormal(const Vector3 &n) {store_Vector3(n,constraints[0].normal);}
	void setDistance(float d) {distance = d;}

	Vector3 getLocalPointA() const {return read_Vector3(localPointA);}
	Vector3 getLocalPointB() const {return read_Vector3(localPointB);}
	Vector3 getLocalVelocityA() const {return read_Vector3(localVelocityA);}
	Vector3 getLocalVelocityB() const {return read_Vector3(localVelocityB);}
	Vector3 getNormal() const {return read_Vector3(constraints[0].normal);}
	float   getDistance() const {return distance;}

	Vector3 getTangent1() const {return read_Vector3(constraints[1].normal);}
	Vector3 getTangent2() const {return read_Vector3(constraints[2].normal);}

	Vector3 getWorldPointA(const Vector3 &pos,const Quat &rot) const
	{
		return pos + rotate(rot,getLocalPointA());
	}
	
	Vector3 getWorldPointB(const Vector3 &pos,const Quat &rot) const
	{
		return pos + rotate(rot,getLocalPointB());
	}

	float getMaxImpulse() const
	{
		return constraints[0].accumImpulse;
	}
};

///////////////////////////////////////////////////////////////////////////////
// Contact Pair (Contact Manifold)

struct ContactPair
{
	uint32_t duration;
	uint16_t numContacts;
	uint16_t stateIndex[2];
	
	float	compositeFriction;
	float	massInvA,massInvB;
	float	inertiaInvA[9];
	float	inertiaInvB[9];

	ContactPoint contactPoints[NUMCONTACTS_PER_BODIES];

	// --------------------------------------------
	
	void reset()
	{
		numContacts = 0;
		duration = 0;
		for(int i=0;i<NUMCONTACTS_PER_BODIES;i++)
			contactPoints[i].reset();
	}

	void exchange()
	{
		for(int i=0;i<numContacts;i++) {
			contactPoints[i].exchange();
		}
		SWAP(uint16_t,stateIndex[0],stateIndex[1]);
	}
	
	ContactPoint &getContactPoint(int idx) {return contactPoints[idx];}

	void setA(int idx,Point3 & localPoint, const Transform3 & objectRelativeTransform, uint8_t primIdx)
	{
		contactPoints[idx].setA(localPoint, objectRelativeTransform, primIdx);
	}

	void setB(int idx,Point3 & localPoint, const Transform3 & objectRelativeTransform,uint8_t primIdx)
	{
		contactPoints[idx].setB(localPoint, objectRelativeTransform, primIdx);
	}

	float getMassInvA() {return massInvA;}
	float getMassInvB() {return massInvB;}

	Matrix3 getInertiaInvA() {return Matrix3(
		read_Vector3(&inertiaInvA[0]),
		read_Vector3(&inertiaInvA[3]),
		read_Vector3(&inertiaInvA[6]));}

	Matrix3 getInertiaInvB() {return Matrix3(
		read_Vector3(&inertiaInvB[0]),
		read_Vector3(&inertiaInvB[3]),
		read_Vector3(&inertiaInvB[6]));}

	void setMassInvA(float massInv) {massInvA = massInv;}
	void setMassInvB(float massInv) {massInvB = massInv;}
	void setInertiaInvA(const Matrix3 &inertiaInv)
	{
		store_Vector3(inertiaInv[0],&inertiaInvA[0]);
		store_Vector3(inertiaInv[1],&inertiaInvA[3]);
		store_Vector3(inertiaInv[2],&inertiaInvA[6]);
	}

	void setInertiaInvB(const Matrix3 &inertiaInv)
	{
		store_Vector3(inertiaInv[0],&inertiaInvB[0]);
		store_Vector3(inertiaInv[1],&inertiaInvB[3]);
		store_Vector3(inertiaInv[2],&inertiaInvB[6]);
	}

	// --------------------------------------------
	
	uint32_t merge(ContactPair &contactPair);
	void refreshContactPoints(const Vector3 &pA,const Quat &qA,const Vector3 &pB,const Quat &qB);
	void removeContactPoint(int idx)
	{
		contactPoints[idx] = contactPoints[numContacts-1];
		numContacts--;
	}
} __attribute__ ((aligned(128)));

int findNearestContactPoint(ContactPoint *cp,int numContacts,const Vector3 &newCP);
int sort4ContactPoints(ContactPoint *cp,const Vector3 &newCP,float newDistance);

#endif /* __CONTACT_H__ */

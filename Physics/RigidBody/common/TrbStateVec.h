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

#ifndef __TRBSTATEVEC_H__
#define __TRBSTATEVEC_H__

#include <stdlib.h>
#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/Base/SimdFunc.h"

// Move Type
enum {
	MoveTypeFixed    = 0,
	MoveTypeActive   = 1,
	MoveTypeKeyframe = 3,
	MoveTypeOneWay   = 5,
	MoveTypeTrigger  = 7,
	MoveTypeCount
};

#define MOVE_TYPE_CAN_SLEEP	((1<<MoveTypeActive)|(1<<MoveTypeKeyframe)|(1<<MoveTypeOneWay))
#define MOVE_TYPE_DYNAMIC	((1<<MoveTypeActive)|(1<<MoveTypeOneWay))
#define MOVE_TYPE_NODYNAMIC	~MOVE_TYPE_DYNAMIC

//
// Rigid Body state
//

class TrbState
{
public:
	TrbState()
	{
		setMoveType(MoveTypeActive);
		contactFilterSelf=contactFilterTarget=0xffffffff;
		deleted = 0;
		sleeping = 0;
		useSleep = 1;
		trbBodyIdx=0;
		sleepCount=0;
		useCcd = 0;
		useContactCallback = 0;
		useSleepCallback = 0;
		linearDamping = 1.0f;
		angularDamping = 0.99f;
	}

	TrbState(const uint8_t m, const Vector3 x, const Quat q, const Vector3 v, const Vector3 omega );
	
	uint16_t	sleepCount;
	uint8_t		moveType;
	uint8_t		deleted            : 1;
	uint8_t		sleeping           : 1;
	uint8_t		useSleep           : 1;
	uint8_t		useCcd		       : 1;
	uint8_t		useContactCallback : 1;
	uint8_t		useSleepCallback   : 1;

	uint16_t	trbBodyIdx;
	uint32_t	contactFilterSelf;
	uint32_t	contactFilterTarget;

	float		center[3];		// AABB center(World)
	float		half[3];		// AABB half(World)

	float		linearDamping;
	float		angularDamping;
	
	float		deltaLinearVelocity[3];
	float		deltaAngularVelocity[3];

	float     fX[3];				// position
	float     fQ[4];				// orientation
	float     fV[3];				// velocity
	float     fOmega[3];			// angular velocity

	inline void setZero();      // Zeroes out the elements
	inline void setIdentity();  // Sets the rotation to identity and zeroes out the other elements

	bool		isDeleted() const {return deleted==1;}

	uint32_t	getContactFilterSelf() const {return contactFilterSelf;}
	void		setContactFilterSelf(uint32_t filter) {contactFilterSelf = filter;}

	uint32_t	getContactFilterTarget() const {return contactFilterTarget;}
	void		setContactFilterTarget(uint32_t filter) {contactFilterTarget = filter;}

	uint8_t		getMoveType() const {return moveType;}
	void		setMoveType(uint8_t t) {moveType = t;sleeping=0;sleepCount=0;}

	uint8_t		getMoveTypeBits() const {return (1<<moveType)|(1<<(moveType+sleeping));}

	bool		isAsleep() const {return sleeping==1;}
	bool		isAwake() const {return sleeping==0;}

	void		wakeup() {sleeping=0;sleepCount=0;}
	void		sleep() {if(useSleep) {sleeping=1;sleepCount=0;}}

	uint8_t		getUseSleep() const {return useSleep;}
	void		setUseSleep(uint8_t b) {useSleep=b;}

	uint8_t		getUseCcd() const {return useCcd;}
	void		setUseCcd(uint8_t b) {useCcd=b;}

	uint8_t		getUseContactCallback() const {return useContactCallback;}
	void		setUseContactCallback(uint8_t b) {useContactCallback=b;}

	uint8_t		getUseSleepCallback() const {return useSleepCallback;}
	void		setUseSleepCallback(uint8_t b) {useSleepCallback=b;}

	void	 	incrementSleepCount() {sleepCount++;}
	void		resetSleepCount() {sleepCount=0;}
	uint16_t	getSleepCount() const {return sleepCount;}

	Vector3 getPosition() const {return read_Vector3(fX);}
	Quat    getOrientation() const {return read_Quat(fQ);}
	Vector3 getLinearVelocity() const {return read_Vector3(fV);}
	Vector3 getAngularVelocity() const {return read_Vector3(fOmega);}
	Vector3 getDeltaLinearVelocity() const {return read_Vector3(deltaLinearVelocity);}
	Vector3 getDeltaAngularVelocity() const {return read_Vector3(deltaAngularVelocity);}

	void setPosition(const Vector3 &pos) {store_Vector3(pos, fX);}
	void setLinearVelocity(const Vector3 &vel) {store_Vector3(vel, fV);}
	void setAngularVelocity(const Vector3 &vel) {store_Vector3(vel, fOmega);}
	void setDeltaLinearVelocity(const Vector3 &vel) {store_Vector3(vel, deltaLinearVelocity);}
	void setDeltaAngularVelocity(const Vector3 &vel) {store_Vector3(vel, deltaAngularVelocity);}
	void setOrientation(const Quat &rot) {store_Quat(rot, fQ);}

	inline void setAuxils(const Vector3 &centerLocal,const Vector3 &halfLocal);
	inline void	setAuxilsCcd(const Vector3 &centerLocal,const Vector3 &halfLocal,float timeStep);
} __attribute__ ((aligned(128)));

inline
TrbState::TrbState(const uint8_t m, const Vector3 x, const Quat q, const Vector3 v, const Vector3 omega)
{
	setMoveType(m);
	fX[0] = x[0];
	fX[1] = x[1];
	fX[2] = x[2];
	fQ[0] = q[0];
	fQ[1] = q[1];
	fQ[2] = q[2];
	fQ[3] = q[3];
	fV[0] = v[0];
	fV[1] = v[1];
	fV[2] = v[2];
	fOmega[0] = omega[0];
	fOmega[1] = omega[1];
	fOmega[2] = omega[2];
	contactFilterSelf=contactFilterTarget=0xffff;
	trbBodyIdx=0;
	sleeping = 0;
	deleted = 0;
	useSleep = 1;
	useCcd = 0;
	useContactCallback = 0;
	useSleepCallback = 0;
	sleepCount=0;
	linearDamping = 1.0f;
	angularDamping = 0.99f;
}

inline void
TrbState::setIdentity()
{
	fX[0] = 0.0f;
	fX[1] = 0.0f;
	fX[2] = 0.0f;
	fQ[0] = 0.0f;
	fQ[1] = 0.0f;
	fQ[2] = 0.0f;
	fQ[3] = 1.0f;
	fV[0] = 0.0f;
	fV[1] = 0.0f;
	fV[2] = 0.0f;
	fOmega[0] = 0.0f;
	fOmega[1] = 0.0f;
	fOmega[2] = 0.0f;
}

inline void
TrbState::setZero()
{
	fX[0] = 0.0f;
	fX[1] = 0.0f;
	fX[2] = 0.0f;
	fQ[0] = 0.0f;
	fQ[1] = 0.0f;
	fQ[2] = 0.0f;
	fQ[3] = 0.0f;
	fV[0] = 0.0f;
	fV[1] = 0.0f;
	fV[2] = 0.0f;
	fOmega[0] = 0.0f;
	fOmega[1] = 0.0f;
	fOmega[2] = 0.0f;
}

inline void
TrbState::setAuxils(const Vector3 &centerLocal,const Vector3 &halfLocal)
{
	Vector3 centerW = getPosition() + rotate(getOrientation(),centerLocal);
	Vector3 halfW = absPerElem(Matrix3(getOrientation())) * halfLocal;
	center[0] = centerW[0];
	center[1] = centerW[1];
	center[2] = centerW[2];
	half[0] = halfW[0];
	half[1] = halfW[1];
	half[2] = halfW[2];
}

inline void
TrbState::setAuxilsCcd(const Vector3 &centerLocal,const Vector3 &halfLocal,float timeStep)
{
	Vector3 centerW = getPosition() + rotate(getOrientation(),centerLocal);
	Vector3 halfW = absPerElem(Matrix3(getOrientation())) * halfLocal;

	Vector3 diffvec = getLinearVelocity()*timeStep;

	Vector3 newCenter = centerW + diffvec;
	Vector3 aabbMin = minPerElem(newCenter - halfW,centerW - halfW);
	Vector3 aabbMax = maxPerElem(newCenter + halfW,centerW + halfW);
	
	centerW = 0.5f * (aabbMin + aabbMax);
	halfW =0.5f * (aabbMax - aabbMin);

	center[0] = centerW[0];
	center[1] = centerW[1];
	center[2] = centerW[2];

	half[0] = halfW[0];
	half[1] = halfW[1];
	half[2] = halfW[2];
}

#endif /* __TRBSTATEVEC_H__ */


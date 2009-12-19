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

#ifndef __JOINT_H__
#define __JOINT_H__

#include "Physics/RigidBody/common/RigidBodyConfig.h"
#include "Physics/RigidBody/common/SubData.h"
#include "Physics/RigidBody/common/ConstraintCache.h"

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

// Joint Type
enum JointType {
	JointTypeBall = 0,	// ボールジョイント
	JointTypeChain,		// チェインジョイント
	JointTypeSlider,	// スライダージョイント
	JointTypeHinge,		// ヒンジジョイント
	JointTypeFix,		// 固定ジョイント
	JointTypeUniversal,	// ユニバーサルジョイント
	JointTypeAnimation,	// アニメーションジョイント
	JointTypeDistance,	// 一定距離に保つジョイント
	JointTypeCount
};

// Joint Axis
enum {
	JointAxisFree = 0x00,
	JointAxisLock = 0x01,
	JointAxisLimit = 0x02,
};

#define GET_AXIS_LOCK(flag,axisId)      ((flag>>(axisId<<2))&0x03)
#define SET_AXIS_LOCK(flag,axisId,lock) ((flag&~(0x03<<(axisId<<2)))|(lock<<(axisId<<2)))

#define CHECK_WARM_STARTING(flag,axisId)   ((flag>>((axisId<<2)+2))&0x01)
#define ENABLE_WARM_STARTING(flag,axisId)  (flag|(0x01<<((axisId<<2)+2)))
#define DISABLE_WARM_STARTING(flag,axisId) (flag&~(0x01<<((axisId<<2)+2)))

struct Joint
{
	uint32_t jointFlag : 26;
	uint8_t  jointType : 4;
	uint16_t stateIndexA;
	uint16_t stateIndexB;

	float lowerLimit[6];
	float upperLimit[6];

	float linearDamping;
	float angularDamping;
	
	float maxLinearImpulse;
	float maxAngularImpulse;
	
	float linearImpulseWeight;
	float angularImpulseWeight;

	float linearBias;
	float angularBias;

	float breakableLimit;
	float massInvA,massInvB;

	SubData subData;
	
	ConstraintCache constraints[6];
	
	Vector3 anchorA;
	Vector3 anchorB;
	
	Matrix3 frameA;
	Matrix3 frameB;
	Matrix3 targetFrame;

	Vector3 localVelocityA,localVelocityB;
	Vector3 rA;
	Vector3 rB;
	Matrix3	inertiaInvA,inertiaInvB;

	void reset()
	{
		jointType = 0;
		jointFlag = 0x2333333;
		for(int i=0;i<6;i++) {
			lowerLimit[i] = upperLimit[i] = 0.0f;
			constraints[i].accumImpulse = 0.0f;
		}
		breakableLimit = 0.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		maxLinearImpulse = 10000.0f;
		maxAngularImpulse = 10000.0f;
		linearImpulseWeight = 1.0f;
		angularImpulseWeight = 1.0f;
		linearBias = 0.2f;
		angularBias = 0.2f;

		localVelocityA = localVelocityB = Vector3(0.0f);
		subData.type = 0;
	}
	
	void enableBreakable() {jointFlag |= (1 << 24);}
	void enableActive() {jointFlag |= (1 << 25);}
	void disableBreakable() {jointFlag = ~(~jointFlag | (1 << 24));}
	void disableActive() {jointFlag = ~(~jointFlag | (1 << 25));}
	
	bool isBreakable() const {return (jointFlag & (1 << 24))!=0;}
	bool isActive() const {return (jointFlag & (1 << 25))!=0;}

	void setFree(int axisId)  {jointFlag = SET_AXIS_LOCK(jointFlag,axisId,JointAxisFree);}
	void setLimit(int axisId) {jointFlag = SET_AXIS_LOCK(jointFlag,axisId,JointAxisLimit);}
	void setLock(int axisId)  {jointFlag = SET_AXIS_LOCK(jointFlag,axisId,JointAxisLock);}

	bool isFree(int axisId)  const {return JointAxisFree  == GET_AXIS_LOCK(jointFlag,axisId);}
	bool isLimit(int axisId) const {return JointAxisLimit == GET_AXIS_LOCK(jointFlag,axisId);}
	bool isLock(int axisId)  const {return JointAxisLock  == GET_AXIS_LOCK(jointFlag,axisId);}

	void enableWarmStarting(int axisId) {jointFlag = ENABLE_WARM_STARTING(jointFlag,axisId);}
	void disableWarmStarting(int axisId) {jointFlag = DISABLE_WARM_STARTING(jointFlag,axisId);}
	bool isWarmStarting(int axisId) const {return 0!=CHECK_WARM_STARTING(jointFlag,axisId);}
} __attribute__ ((aligned(128)));

#endif

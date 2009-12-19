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

#ifndef __RIGIDBODYIO_H__
#define __RIGIDBODYIO_H__

#ifndef WIN32

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/SortCommon.h"

///////////////////////////////////////////////////////////////////////////////
// Event

enum {
	// Broadphase Event
	BROADPHASE_CALCVARIANCE = 0,
	BROADPHASE_ASSIGNSTATES,
	BROADPHASE_DETECTPAIRS_MOV,
	BROADPHASE_DETECTPAIRS_FIX,
	BROADPHASE_MERGEPAIRS,
	BROADPHASE_REFRESHCONTACTPAIRS,
	BROADPHASE_ADDNEWPAIRS,
	BROADPHASE_INTEGRATE,
	BROADPHASE_SLEEP,
	BROADPHASE_SORT,
	FIND_AABB_OVERLAP,

	// Solver Event
	SOLVER_SPLIT_CONSTRAINTS= 0,
	SOLVER_CREATE_JOINT_PAIRS,
	SOLVER_CONSTRAINT,
	SOLVER_CONSTRAINT_EX,
	SOLVER_POSTRESPONSE,
};

///////////////////////////////////////////////////////////////////////////////
// IO Parameter

struct IOParamAssignStates {
	uint32_t statesAddr;
	uint32_t numStates;
	uint32_t batchStartState;
	uint32_t numBatchStates;
	uint32_t movAabbAddr;
	uint32_t fixAabbAddr;
	uint32_t numMovAabb;
	uint32_t numFixAabb;
	uint32_t worldVolumeAddr;
	uint32_t chkAxis;
	float	 timeStep;
	Vector3 s;
	Vector3 s2;
} __attribute__ ((aligned(16)));

struct IOParamDetectPairs {
	uint32_t movAabbAddr;
	uint32_t fixAabbAddr;
	uint32_t numMovAabb;
	uint32_t numFixAabb;
	uint32_t tmpSortsAddr;
	uint32_t numTmpSorts;
	uint32_t maxSorts;
	uint32_t chkAxis;
	uint32_t nonContactPairAddr;
} __attribute__ ((aligned(16)));

struct IOParamMergePairs {
	uint32_t newSortsAddr;
	uint32_t numNewSorts;
	uint32_t oldSortsAddr;
	uint32_t numOldSorts;
	uint32_t statesAddr;
	uint32_t numStates;
	uint32_t maxInstances;
} __attribute__ ((aligned(16)));

struct IOParamSort {
	uint32_t numSpu;
	uint32_t buffAddr;
	uint32_t sortsAddr;
	uint32_t numSorts;
} __attribute__ ((aligned(16)));

struct IOParamIntegrate {
	uint32_t startIdx;
	uint32_t statesAddr;
	uint32_t numStates;
	uint32_t prevAddr;
	uint32_t forcesAddr;
	uint32_t bodiesAddr;
	uint32_t collsAddr;
	float    timeStep;
	bool	 ccdEnable;
	Vector3  gravity;
	
	bool  sleepEnable;
	uint16_t sleepCount;
	uint16_t sleepInterval;
	float sleepLinearVelocity;
	float sleepAngularVelocity;
	float wakeLinearVelocity;
	float wakeAngularVelocity;
	uint32_t worldSleepCount;
	
	bool lastSubStep;
} __attribute__ ((aligned(16)));

struct IOParamRefreshPairs {
	uint32_t numSpu;

	uint32_t statesAddr;		// 剛体インスタンス
	uint32_t numStates;

	uint32_t buffAddr;
	uint32_t sortsAddr;
	uint32_t contactPairsAddr;
	uint32_t numContactPairs;

	uint32_t startBatch;		// 開始コンタクトペア
	uint32_t numBatch;			// 処理数
	
	uint32_t numRemovedPairs;	// 削除されたコンタクトペア数
} __attribute__ ((aligned(16)));

struct IOParamAddNewPairs {
	uint32_t startPair;
	uint32_t batchPair;
	uint32_t numPairs;
	uint32_t contactsAddr;
	uint32_t pairsAddr;
	uint32_t newPairsAddr;
} __attribute__ ((aligned(16)));

struct IOParamFindAabbOverlap {
	uint32_t chkAxis;
	SortData inAabb;
	uint32_t inTarget;
	uint32_t inSelf;
	uint32_t aabbAddr;
	uint32_t numAabb;
	uint32_t overlappedAddr;
	uint32_t numOverlapped;
	uint32_t maxOverlapped;
} __attribute__ ((aligned(16)));

struct IOParamCollision {
	uint32_t statesAddr;		// 剛体インスタンス
	uint32_t bodiesAddr;		// 剛体
	uint32_t collsAddr;			// 形状
	uint32_t numStates;
	uint32_t numBodies;

	uint32_t sortsAddr;
	uint32_t contactPairsAddr;
	uint32_t numContactPairs;

	uint32_t startBatch;		// 開始コンタクトペア
	uint32_t numBatch;			// 処理数

	bool ccdEnable;				// 連続的衝突判定
	
	float	timeStep;			// タイムステップ
} __attribute__ ((aligned(16)));

struct IOParamSplitConstraints {
	uint32_t numSpu;
	uint32_t numInstances;
	uint32_t numPairs;
	uint32_t pairsAddr;
	uint32_t solverInfoAddr;
	uint32_t solverGroupsAddr;
} __attribute__ ((aligned(16)));

struct IOParamCreateJointPairs {
	uint32_t startJoint;
	uint32_t batchJoint;
	uint32_t jointsAddr;
	uint32_t pairsAddr;
	uint32_t statesAddr;
	uint32_t numActiveJoints;
} __attribute__ ((aligned(16)));

struct IOParamSolver {
	uint32_t statesAddr;
	uint32_t bodiesAddr;
	uint32_t collsAddr;

	uint32_t contactSolverInfoAddr;
	uint32_t contactGroupsAddr;
	uint32_t contactPairsAddr;
	uint32_t contactsAddr;
	uint32_t numCollIteration;

	uint32_t jointSolverInfoAddr;
	uint32_t jointGroupsAddr;
	uint32_t jointPairsAddr;
	uint32_t jointsAddr;
	uint32_t numJointIteration;

	uint32_t numSPU;
	uint32_t numStates;
	uint32_t numBodies;
	uint32_t numContactPairs;
	uint32_t numJoints;

	float	timeStep;			// タイムステップ
	float	separateBias;		// 貫通回避バイアス
	bool	deformMeshEnable;	// 変形メッシュ
} __attribute__ ((aligned(16)));

struct IOParamPostResponse {
	uint32_t statesAddr;
	uint32_t numStates;
	float	 maxLinearVelocity;
	float	 maxAngularVelocity;
} __attribute__ ((aligned(16)));

#endif

#endif /* __RIGIDBODYCOLLIO_H__ */


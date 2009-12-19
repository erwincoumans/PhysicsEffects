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
#include "Physics/RigidBody/RigidBodies.h"
#include "Physics/RigidBody/common/closestContact.h"
#include "Physics/RigidBody/common/collidableTable.h"
#include "Physics/Base/PerfCounter.h"
#include "Physics/Base/TestAABB.h"
#include "Physics/Sort/Quicksort.ppu.h"

#define ALLOCATE(align,size) mPool->allocate(size,align==128?HeapManager::ALIGN128:HeapManager::ALIGN16);
#define DEALLOCATE(ptr) if(ptr) {mPool->deallocate(((void*)ptr));ptr=NULL;}

///////////////////////////////////////////////////////////////////////////////
// PPU シミュレーション

void RigidBodies::ppuSimulate(float timeStep,uint32_t flag)
{
	debugFlag = flag;

	PerfCounter pc;

	memcpy(statesBuffer[writeBuffer],statesBuffer[readBuffer],sizeof(TrbState)*numInstances);
	states = statesBuffer[writeBuffer];

	float subTimeStep = timeStep / (float)worldProperty.subStepCount;
	for(uint32_t substep=0;substep<worldProperty.subStepCount;substep++) {
		pc.countBegin("broad phase");
		broadPhase(subTimeStep);
		pc.countEnd();

		pc.countBegin("detect collisions");
		detectCollisions(subTimeStep);
		pc.countEnd();

		pc.countBegin("refresh collisions");
		if(numContactPairs > 0) {
			uint32_t numRemovedContactPairs = refreshContactPairs();
			quickSort(sortedContactPairs,0,numContactPairs-1);
			numContactPairs -= numRemovedContactPairs;
		}
		pc.countEnd();
		
		if(worldProperty.deformMeshEnable) {
			pc.countBegin("update facet local");
			updateContactPoints(subTimeStep);
			updateJointPoints(subTimeStep);
			pc.countEnd();
		}

		pc.countBegin("response");
		solveConstraints(subTimeStep);
		pc.countEnd();

		pc.countBegin("sleep");
		if(worldProperty.sleepEnable) sleepOrWakeup();
		pc.countEnd();

		pc.countBegin("sleep callback");
		throwSleepCallback();
		pc.countEnd();

		pc.countBegin("integrate");
		integrate(subTimeStep,((int)substep==worldProperty.subStepCount-1));
		pc.countEnd();

		pc.countBegin("contact callback");
		throwContactCallback();
		pc.countEnd();
	}
}

void RigidBodies::mergePairs(SortData *newPairs,uint32_t &numNewPairs)
{
	int numRemoved = 0;

	uint32_t oldIdx = 0,newIdx = 0;

	uint32_t asleepMask = ( (1<<(MoveTypeActive+1))|(1<<(MoveTypeOneWay+1)) );

	for(;newIdx<numNewPairs;newIdx++) {
		if((1<<MovA(newPairs[newIdx])) & asleepMask) {
			TrbState &stateA = states[StateA(newPairs[newIdx])];
			stateA.wakeup();
		}
		if((1<<MovB(newPairs[newIdx])) & asleepMask) {
			TrbState &stateB = states[StateB(newPairs[newIdx])];
			stateB.wakeup();
		}

		bool loopOut = false;
		do {
			if(oldIdx >= numContactPairs) {
				loopOut = true;
			}
			else if(Key(newPairs[newIdx]) > Key(sortedContactPairs[oldIdx])) {
				oldIdx++;
			}
			else if(Key(newPairs[newIdx]) == Key(sortedContactPairs[oldIdx])) {
				setFlag(sortedContactPairs[oldIdx],1); // 更新したことを示す
				setKey(newPairs[newIdx],NULL_KEY);
				oldIdx++;
				numRemoved++;
				loopOut = true;
			}
			else {
				// 新規追加
				loopOut = true;
			}
		} while(!loopOut);
	}

	quickSort((SortData*)newPairs,0,numNewPairs-1);

	numNewPairs -= numRemoved;
}

void RigidBodies::broadPhase(float timeStep)
{
	(void) timeStep;

	PerfCounter pc;

	int chkAxis = 0; // 判定軸
	uint32_t numMovAabb = 0;
	uint32_t numFixAabb = 0;

	SortData *movAabbArray; // アクティブ剛体AABBの各XYZ軸配列
	SortData *fixAabbArray; // 固定剛体AABBの各XYZ軸配列
	movAabbArray = (SortData*)ALLOCATE(16,sizeof(SortData)*numInstances);
	fixAabbArray = (SortData*)ALLOCATE(16,sizeof(SortData)*numInstances);

	Vector3 worldHalf(worldVolume.extent);

	//----------------------------------------------------------------------------
	// 剛体の分散を調べる

	{
		Vector3 s(0.0f),s2(0.0f);
		for(uint32_t i=0;i<numInstances;i++) {
			TrbState &state = states[i];

			Vector3 stateCenter(state.center[0],state.center[1],state.center[2]);
			s += stateCenter;
			s2 += mulPerElem(stateCenter,stateCenter);
		}

		// 分散の計算
		Vector3 v = s2 - mulPerElem(s,s) / (float)numInstances;
		if(v[1] > v[0]) chkAxis = 1;
		if(v[2] > v[chkAxis]) chkAxis = 2;
	}

	//----------------------------------------------------------------------------
	// 剛体のAABBを各XYZ軸に対して並べる
	
	pc.countBegin("assign states");
	{
		Vector3 s(0.0f),s2(0.0f);
		for(uint32_t i=0;i<numInstances;i++) {
			TrbState &state = states[i];
			
			if(state.isDeleted()) continue;
			
			// AABB
			Vector3 stateCenter = read_Vector3(state.center);
			Vector3 stateHalf = read_Vector3(state.half);
			Vector3 aabbMin = stateCenter - stateHalf;
			Vector3 aabbMax = stateCenter + stateHalf;

			// 範囲チェック（ブロードフェーズ範囲外の剛体はスリープへ移行）
			Vector3 checkInWorld = absPerElem(stateCenter-worldVolume.origin) - (stateHalf+worldHalf);
			if(checkInWorld[0] > 0.0f || checkInWorld[1] > 0.0f || checkInWorld[2] > 0.0f) {
				if(state.getMoveTypeBits() & MOVE_TYPE_DYNAMIC) {
					state.setLinearVelocity(Vector3(0));
					state.setAngularVelocity(Vector3(0));
					state.sleep();
				}
				continue;
			}

			// ブロードフェーズ領域に正規化
			VecInt3 aabbMinL;
			VecInt3 aabbMaxL;
			
			aabbMinL = worldVolume.worldToLocalPosition(aabbMin);
			aabbMaxL = worldVolume.worldToLocalPosition(aabbMax);

			// 配列に追加
			SortData &aabb = (state.getMoveType()==MoveTypeFixed || state.isAsleep() )?fixAabbArray[numFixAabb]:movAabbArray[numMovAabb];

			setKey(aabb,aabbMinL.get(chkAxis));

			setXMin(aabb,aabbMinL.getX());
			setXMax(aabb,aabbMaxL.getX());
			setYMin(aabb,aabbMinL.getY());
			setYMax(aabb,aabbMaxL.getY());
			setZMin(aabb,aabbMinL.getZ());
			setZMax(aabb,aabbMaxL.getZ());

			setStateId(aabb,i);
			setBodyId(aabb,state.trbBodyIdx);
			setMovType(aabb,state.moveType+state.sleeping); // set index for func table
			setSelf(aabb,state.getContactFilterSelf());
			setTarget(aabb,state.getContactFilterTarget());
			setCallback(aabb,state.getUseContactCallback());
			
			if(state.getMoveType()==MoveTypeFixed || state.isAsleep() )
				numFixAabb++;
			else
				numMovAabb++;
		}

		quickSort(movAabbArray,0,numMovAabb-1);
		quickSort(fixAabbArray,0,numFixAabb-1);
	}
	pc.countEnd();

	//----------------------------------------------------------------------------
	// 衝突候補ペアチェック

	pc.countBegin("detect pairs");
	uint32_t numNewPairs = 0;
	SortData *newPairs = (SortData*)ALLOCATE(128,sizeof(SortData)*worldProperty.maxContactPairs);

	for(uint32_t i=0;i<numMovAabb;i++) {
		uint32_t slfA,tgtA;
		uint16_t stateIndexA,movA;
		uint16_t aabbMaxA[3];
		SortData aabbA = movAabbArray[i];

		stateIndexA = StateId(aabbA);
		movA = MovType(aabbA);
		slfA = Self(aabbA);
		tgtA = Target(aabbA);

		aabbMaxA[0] = XMax(aabbA);
		aabbMaxA[1] = YMax(aabbA);
		aabbMaxA[2] = ZMax(aabbA);
		
		for(uint32_t j=i+1;j<numMovAabb;j++) {
			uint32_t slfB,tgtB;
			uint16_t stateIndexB,movB;
			uint16_t aabbMinB[3];
			SortData aabbB = movAabbArray[j];

			stateIndexB = StateId(aabbB);
			movB = MovType(aabbB);
			slfB = Self(aabbB);
			tgtB = Target(aabbB);

			aabbMinB[0] = XMin(aabbB);
			aabbMinB[1] = YMin(aabbB);
			aabbMinB[2] = ZMin(aabbB);

			if(aabbMaxA[chkAxis] < aabbMinB[chkAxis]) {
				break;
			}

			if(collidableTable[movA][movB] && isCollidable(stateIndexA,slfA,tgtA,stateIndexB,slfB,tgtB) && testAABB16(aabbA,aabbB)) {
				setStatePair(newPairs[numNewPairs],stateIndexA,stateIndexB);
				setBodyA(newPairs[numNewPairs],BodyId(aabbA));
				setBodyB(newPairs[numNewPairs],BodyId(aabbB));
				setMovA(newPairs[numNewPairs],movA);
				setMovB(newPairs[numNewPairs],movB);
				setFlag(newPairs[numNewPairs],1);
				setCallbackFlag(newPairs[numNewPairs],Callback(aabbA)|Callback(aabbB));
				numNewPairs++;
			}
		}
	}

	for(uint32_t i=0;i<numMovAabb;i++) {
		uint32_t slfA,tgtA;
		uint16_t stateIndexA,movA;
		uint16_t aabbMaxA[3];
		SortData aabbA = movAabbArray[i];

		stateIndexA = StateId(aabbA);
		movA = MovType(aabbA);
		slfA = Self(aabbA);
		tgtA = Target(aabbA);

		aabbMaxA[0] = XMax(aabbA);
		aabbMaxA[1] = YMax(aabbA);
		aabbMaxA[2] = ZMax(aabbA);

		for(uint32_t j=0;j<numFixAabb;j++) {
			uint32_t slfB,tgtB;
			uint16_t stateIndexB,movB;
			uint16_t aabbMinB[3];
			SortData aabbB = fixAabbArray[j];

			stateIndexB = StateId(aabbB);
			movB = MovType(aabbB);
			slfB = Self(aabbB);
			tgtB = Target(aabbB);

			aabbMinB[0] = XMin(aabbB);
			aabbMinB[1] = YMin(aabbB);
			aabbMinB[2] = ZMin(aabbB);

			if(aabbMaxA[chkAxis] < aabbMinB[chkAxis]) {
				break;
			}

			if(collidableTable[movA][movB] && isCollidable(stateIndexA,slfA,tgtA,stateIndexB,slfB,tgtB) && testAABB16(aabbA,aabbB)) {
				setStatePair(newPairs[numNewPairs],stateIndexA,stateIndexB);
				setBodyA(newPairs[numNewPairs],BodyId(aabbA));
				setBodyB(newPairs[numNewPairs],BodyId(aabbB));
				setMovA(newPairs[numNewPairs],movA);
				setMovB(newPairs[numNewPairs],movB);
				setFlag(newPairs[numNewPairs],1);
				setCallbackFlag(newPairs[numNewPairs],Callback(aabbA)|Callback(aabbB));
				numNewPairs++;
			}
		}
	}

	if(numNewPairs > 0) {
		quickSort((SortData*)newPairs,0,numNewPairs-1);
		mergePairs(newPairs,numNewPairs);

		PFX_ASSERT(numContactPairs + numNewPairs <= worldProperty.maxContactPairs);

		for(uint32_t i=0;i<numNewPairs;i++) {
			SortData &sort = sortedContactPairs[numContactPairs++];
			int pairIdx = Pair(sort);
			ContactPair &pair = contactPairs[pairIdx];
			pair.duration = 0;
			pair.numContacts = 0;
			pair.stateIndex[0] = StateA(newPairs[i]);
			pair.stateIndex[1] = StateB(newPairs[i]);
			sort = newPairs[i];
			setPair(sort,pairIdx);
			setFlag(sort,1); // 更新したことを示す
		}
	}

	DEALLOCATE(newPairs);
	pc.countEnd();

	DEALLOCATE(fixAabbArray);
	DEALLOCATE(movAabbArray);
}

void RigidBodies::detectCollisions(float timeStep)
{
	//----------------------------------------------------------------------------
	// 剛体ペアを順番に衝突判定

	for(uint32_t i=0;i<numContactPairs;i++) {
		if(Flag(sortedContactPairs[i]) == 0)
			continue;

		ContactPair &curPair = contactPairs[Pair(sortedContactPairs[i])];

		TrbState &stateA = states[curPair.stateIndex[0]];
		TrbState &stateB = states[curPair.stateIndex[1]];

		Transform3 tA0(stateA.getOrientation(), stateA.getPosition());
		Transform3 tB0(stateB.getOrientation(), stateB.getPosition());

		CollObject *collA = bodies[stateA.trbBodyIdx].getCollObject();
		CollObject *collB = bodies[stateB.trbBodyIdx].getCollObject();

		ContactPair contactPair;
		contactPair.reset();
		contactPair.stateIndex[0] = curPair.stateIndex[0];
		contactPair.stateIndex[1] = curPair.stateIndex[1];

		bool ret = false;
		if(worldProperty.ccdEnable) {
			Transform3 tA1 = integrateTransform(timeStep,tA0,stateA.getLinearVelocity(),stateA.getAngularVelocity());
			Transform3 tB1 = integrateTransform(timeStep,tB0,stateB.getLinearVelocity(),stateB.getAngularVelocity());
			ret = findContactCCD(contactPair,
					*(collA), tA0, tA1, stateA.getUseCcd(),
					*(collB), tB0, tB1, stateB.getUseCcd(),
					CONTACT_EPSILON);
		}
		else {
			ret = closestContact(contactPair,
					*(collA), tA0,
					*(collB), tB0,
					CONTACT_EPSILON);
		}

		if(ret) {
			curPair.merge(contactPair);
		}
	}
}

uint32_t RigidBodies::refreshContactPairs()
{
	uint32_t numRemovedContactPairs = 0;
	uint32_t asleepMask = ((1<<(MoveTypeActive+1))|(1<<(MoveTypeOneWay+1)));

	for(uint32_t i=0;i<numContactPairs;i++) {
		ContactPair &curPair = contactPairs[Pair(sortedContactPairs[i])];

		TrbState &stateA = states[curPair.stateIndex[0]];
		TrbState &stateB = states[curPair.stateIndex[1]];

		// Update pair info
		setFlag(sortedContactPairs[i],0);
		setMovA(sortedContactPairs[i],stateA.moveType+stateA.sleeping);
		setMovB(sortedContactPairs[i],stateB.moveType+stateB.sleeping);

		if(curPair.numContacts == 0 ||
			!((stateA.getContactFilterSelf()&stateB.getContactFilterTarget())&&(stateA.getContactFilterTarget()&stateB.getContactFilterSelf())) ) {

			if(stateA.getMoveTypeBits() & asleepMask) {
				stateA.wakeup();
			}
			if(stateB.getMoveTypeBits() & asleepMask) {
				stateB.wakeup();
			}
			setKey(sortedContactPairs[i],NULL_KEY);
			numRemovedContactPairs++;
		}

		curPair.refreshContactPoints(stateA.getPosition(),stateA.getOrientation(),stateB.getPosition(),stateB.getOrientation());
	}

	return numRemovedContactPairs;
}

void RigidBodies::solveConstraints(float timeStep)
{
	preResponse(timeStep);
	preJoint(timeStep);
	
	int iteration = PFX_MAX(worldProperty.contactIteration,worldProperty.jointIteration);
	for(int i=0;i<iteration;i++) {
		if((iteration - i) <= worldProperty.contactIteration) {
			applyImpulse(timeStep);
		}
		if((iteration - i) <= worldProperty.jointIteration) {
			applyJoint(timeStep);
		}
	}

	postResponse();
}

void RigidBodies::setupAabbOverlapUtil()
{
	mNumAabbArray = 0;
	mAabbArray = (SortData*)ALLOCATE(16,sizeof(SortData)*numInstances);
	
	Vector3 worldHalf(worldVolume.extent);
	
	//----------------------------------------------------------------------------
	// 剛体の分散を調べる

	{
		Vector3 s(0.0f),s2(0.0f);
		for(uint32_t i=0;i<numInstances;i++) {
			TrbState &state = statesBuffer[readBuffer][i];

			Vector3 stateCenter(state.center[0],state.center[1],state.center[2]);
			s += stateCenter;
			s2 += mulPerElem(stateCenter,stateCenter);
		}

		// 分散の計算
		Vector3 v = s2 - mulPerElem(s,s) / (float)numInstances;
		if(v[1] > v[0]) mCheckAxis = 1;
		if(v[2] > v[mCheckAxis]) mCheckAxis = 2;
	}

	//----------------------------------------------------------------------------
	// 剛体のAABBを各XYZ軸に対して並べる
	
	{
		Vector3 s(0.0f),s2(0.0f);
		for(uint32_t i=0;i<numInstances;i++) {
			TrbState &state = statesBuffer[readBuffer][i];

			if(state.isDeleted()) continue;

			// AABB
			Vector3 stateCenter = read_Vector3(state.center);
			Vector3 stateHalf = read_Vector3(state.half);
			Vector3 aabbMin = stateCenter - stateHalf;
			Vector3 aabbMax = stateCenter + stateHalf;

			// 範囲チェック
			Vector3 checkInWorld = absPerElem(stateCenter-worldVolume.origin) - (stateHalf+worldHalf);
			if(checkInWorld[0] > 0.0f || checkInWorld[1] > 0.0f || checkInWorld[2] > 0.0f) {
				continue;
			}

			// ブロードフェーズ領域に正規化
			VecInt3 aabbMinL;
			VecInt3 aabbMaxL;
			
			aabbMinL = worldVolume.worldToLocalPosition(aabbMin);
			aabbMaxL = worldVolume.worldToLocalPosition(aabbMax);

			// 配列に追加
			SortData &aabb = mAabbArray[mNumAabbArray++];

			setKey(aabb,aabbMinL.get(mCheckAxis));

			setXMin(aabb,aabbMinL.getX());
			setXMax(aabb,aabbMaxL.getX());
			setYMin(aabb,aabbMinL.getY());
			setYMax(aabb,aabbMaxL.getY());
			setZMin(aabb,aabbMinL.getZ());
			setZMax(aabb,aabbMaxL.getZ());

			setStateId(aabb,i);
			setSelf(aabb,state.getContactFilterSelf());
			setTarget(aabb,state.getContactFilterTarget());
		}

		quickSort(mAabbArray,0,mNumAabbArray-1);
	}
}

void RigidBodies::finalizeAabbOverlapUtil()
{
	DEALLOCATE(mAabbArray);
}

void RigidBodies::ppuFindAabbOverlap(Vector3 aabbMin,Vector3 aabbMax,uint32_t self,uint32_t target,AabbOverlapCallback *findAabbOverlap)
{
	VecInt3 inAabbMin = worldVolume.worldToLocalPosition(aabbMin);
	VecInt3 inAabbMax = worldVolume.worldToLocalPosition(aabbMax);
	
	SortData inAabb(0);
	setXMin(inAabb,inAabbMin.getX());
	setXMax(inAabb,inAabbMax.getX());
	setYMin(inAabb,inAabbMin.getY());
	setYMax(inAabb,inAabbMax.getY());
	setZMin(inAabb,inAabbMin.getZ());
	setZMax(inAabb,inAabbMax.getZ());
	
	for(int i=0;i<mNumAabbArray;i++) {
		uint32_t slfA,tgtA;
		uint16_t stateIndexA;
		uint16_t aabbMinA[3];
		SortData aabbA = mAabbArray[i];

		stateIndexA = StateId(aabbA);
		slfA = Self(aabbA);
		tgtA = Target(aabbA);

		aabbMinA[0] = XMin(aabbA);
		aabbMinA[1] = YMin(aabbA);
		aabbMinA[2] = ZMin(aabbA);
		
		if(inAabbMax.get(mCheckAxis) < aabbMinA[mCheckAxis]) {
			break;
		}

		if(((slfA&target) && (tgtA&self)) && testAABB16(aabbA,inAabb)) {
			findAabbOverlap->onOverlap(stateIndexA);
		}
	}
}

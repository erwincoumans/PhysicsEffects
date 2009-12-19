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

#include <string.h>
#include "Physics/RigidBody/RigidBodies.h"
#include "Physics/RigidBody/common/intersectFunction.h"
#include "Physics/Base/TestAABB.h"
#include "Physics/Sort/Quicksort.ppu.h"
#include "Physics/RayCast/RayCast.h"

#define ALLOCATE(align,size) mPool->allocate(size,align==128?HeapManager::ALIGN128:HeapManager::ALIGN16);
#define DEALLOCATE(ptr) if(ptr) {mPool->deallocate(((void*)ptr));ptr=NULL;}

#define REPSILON 0.00001f

///////////////////////////////////////////////////////////////////////////////
// Initialize

#ifndef WIN32
RayCast::RayCast(RayCastTaskMulti *RCTask,int mRCTaskID,HeapManager *pool)
{
	mRCTask = RCTask;
	mRCTaskID = mRCTaskID;
#else
RayCast::RayCast(HeapManager *pool)
{
#endif
	mRigidBodies = NULL;
	states = NULL;
	colls = NULL;
	numInstances = 0;

	nonContactFlag = NULL;
	
	// プールメモリ
	mPool = pool;
}

void RayCast::reset()
{
	// バッファの確保
	deallocateBuffers();
	allocateBuffers();

	clearNonContactFlag();

	debugFlag = 0;
}

void RayCast::allocateBuffers()
{
	sizeContactTable = raycastProperty.maxInstances * raycastProperty.maxRayGroups;
	sizeContactTable = PFX_ALIGN128((sizeContactTable+31)/32,sizeof(uint32_t));
	nonContactFlag = (uint32_t*)ALLOCATE(128,sizeof(uint32_t)*sizeContactTable);
	printBufSize();
}

void RayCast::deallocateBuffers()
{
	DEALLOCATE(nonContactFlag);
}

void RayCast::printBufSize()
{
	#define PRINTBUF(val,type,count) \
		PRINTF(#val " : sizeof(" #type ") %d bytes * %d = %d bytes\n",sizeof(type),count,sizeof(type)*count);\
		allocatedBytes+=sizeof(type)*count;

	{
		int allocatedBytes = 0;
		PRINTF(" --- RayCast XDR buffers ---\n");
		PRINTBUF(nonContactFlag,uint32_t,sizeContactTable);
		PRINTBUF(aabb,SortData,raycastProperty.maxInstances*3);
		PRINTF("total : %d bytes\n",allocatedBytes);
	}

	#undef PRINTBUF
}

uint32_t RayCast::calcBufSize()
{
	#define CALCBUF(type,count) (sizeof(type)*count)

	int bufSize = 0;
	int contactTable = raycastProperty.maxInstances * raycastProperty.maxRayGroups;
	contactTable = PFX_ALIGN128((contactTable+31)/32,sizeof(uint32_t));

	bufSize += CALCBUF(uint32_t,contactTable);

	return bufSize;

	#undef CALCBUF
}

void RayCast::setupWorldSize()
{
	if(raycastProperty.useRigidBodyWorldSize) {
		worldVolume.setWorldSize(mRigidBodies->worldProperty.worldCenter,mRigidBodies->worldProperty.worldExtent);
	}
	else {
		worldVolume.setWorldSize(raycastProperty.worldCenter,raycastProperty.worldExtent);
	}
}

void RayCast::getWorldSize(Vector3 &center,Vector3 &extent)
{
	center = raycastProperty.worldCenter;
	extent = raycastProperty.worldExtent;
}

void RayCast::setWorldSize(const Vector3 &center,const Vector3 &extent)
{
	raycastProperty.worldCenter = center;
	raycastProperty.worldExtent = extent;
	setupWorldSize();
}

///////////////////////////////////////////////////////////////////////////////
// NonContactFlag

void RayCast::appendNonContactPair(uint8_t rayGroup,uint16_t rigidbodyIndex)
{
	uint32_t idx = rayGroup * raycastProperty.maxRayGroups + rigidbodyIndex;
	nonContactFlag[idx>>5] |= 1L << (idx & 31);
}

void RayCast::removeNonContactPair(uint8_t rayGroup,uint16_t rigidbodyIndex)
{
	uint32_t idx = rayGroup * raycastProperty.maxRayGroups + rigidbodyIndex;
	nonContactFlag[idx>>5] = nonContactFlag[idx>>5] & ~(1L << (idx & 31));
}

void RayCast::clearNonContactFlag()
{
	memset(nonContactFlag,0,sizeof(uint32_t)*sizeContactTable);
}

///////////////////////////////////////////////////////////////////////////////
// Attach / Detach RigidBodies

void RayCast::attachRigidBodies(RigidBodies *rb)
{
	PFX_ASSERT(rb);
	
	mRigidBodies = rb;
}

void RayCast::detachRigidBodies()
{
	mRigidBodies = NULL;
}

///////////////////////////////////////////////////////////////////////////////
// Broad Phase

void RayCast::broadPhase(SortData *aabbArray[3],int &numAabb)
{
	PerfCounter pc;

	//----------------------------------------------------------------------------
	// 剛体のAABBを各XYZ軸に対して並べる

	numAabb = 0;

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

			// 範囲チェック
			Vector3 worldHalf(worldVolume.extent);
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
			SortData &aabbX = aabbArray[0][numAabb];
			setKey(aabbX,aabbMinL.getX());
			setXMin(aabbX,aabbMinL.getX());
			setXMax(aabbX,aabbMaxL.getX());
			setYMin(aabbX,aabbMinL.getY());
			setYMax(aabbX,aabbMaxL.getY());
			setZMin(aabbX,aabbMinL.getZ());
			setZMax(aabbX,aabbMaxL.getZ());
			setStateId(aabbX,i);
			setBodyId(aabbX,state.trbBodyIdx);
			setMovType(aabbX,state.getMoveType());
			setSelf(aabbX,state.getContactFilterSelf());
			setTarget(aabbX,state.getContactFilterTarget());

			SortData &aabbY = aabbArray[1][numAabb];
			setKey(aabbY,aabbMinL.getY());
			setXMin(aabbY,aabbMinL.getX());
			setXMax(aabbY,aabbMaxL.getX());
			setYMin(aabbY,aabbMinL.getY());
			setYMax(aabbY,aabbMaxL.getY());
			setZMin(aabbY,aabbMinL.getZ());
			setZMax(aabbY,aabbMaxL.getZ());
			setStateId(aabbY,i);
			setBodyId(aabbY,state.trbBodyIdx);
			setMovType(aabbY,state.getMoveType());
			setSelf(aabbY,state.getContactFilterSelf());
			setTarget(aabbY,state.getContactFilterTarget());

			SortData &aabbZ = aabbArray[2][numAabb];
			setKey(aabbZ,aabbMinL.getZ());
			setXMin(aabbZ,aabbMinL.getX());
			setXMax(aabbZ,aabbMaxL.getX());
			setYMin(aabbZ,aabbMinL.getY());
			setYMax(aabbZ,aabbMaxL.getY());
			setZMin(aabbZ,aabbMinL.getZ());
			setZMax(aabbZ,aabbMaxL.getZ());
			setStateId(aabbZ,i);
			setBodyId(aabbZ,state.trbBodyIdx);
			setMovType(aabbZ,state.getMoveType());
			setSelf(aabbZ,state.getContactFilterSelf());
			setTarget(aabbZ,state.getContactFilterTarget());

			numAabb++;
		}

		quickSort(aabbArray[0],0,numAabb-1);
		quickSort(aabbArray[1],0,numAabb-1);
		quickSort(aabbArray[2],0,numAabb-1);
	}
	pc.countEnd();
}

///////////////////////////////////////////////////////////////////////////////
// PPU Ray Cast

void RayCast::setupRayCast()
{
	PFX_ASSERT(mRigidBodies);

	states = mRigidBodies->statesBuffer[mRigidBodies->readBuffer];
	colls = mRigidBodies->collObjs;
	numInstances = mRigidBodies->numInstances;

	setupWorldSize();

	aabbArray[0] = (SortData*)ALLOCATE(16,sizeof(SortData)*numInstances);
	aabbArray[1] = (SortData*)ALLOCATE(16,sizeof(SortData)*numInstances);
	aabbArray[2] = (SortData*)ALLOCATE(16,sizeof(SortData)*numInstances);
	
	broadphaseOk = false;
}

void RayCast::finalizeRayCast()
{
	DEALLOCATE(aabbArray[2]);
	DEALLOCATE(aabbArray[1]);
	DEALLOCATE(aabbArray[0]);
}

void RayCast::ppuRayCast(Ray *rays,int numRay,uint32_t flag)
{
	debugFlag = flag;
	PerfCounter pc;

	pc.countBegin("raycast");

	// -------------------------------------------------
	// ブロードフェーズ

	if(!broadphaseOk) {
		broadPhase(aabbArray,numAabb);
		broadphaseOk = true;
	}

	//PRINTF("::::: aabb :::::\n");
	//for(int i=0;i<numAabb;i++) {
	//	PRINTF("state %5u min %5u %5u %5u max %5u %5u %5u\n",
	//		StateId(aabbArray[0][i]),
	//		XMin(aabbArray[0][i]),YMin(aabbArray[0][i]),ZMin(aabbArray[0][i]),
	//		XMax(aabbArray[0][i]),YMax(aabbArray[0][i]),ZMax(aabbArray[0][i]));
	//}

	// -------------------------------------------------
	// レイキャスト

	for(int r=0;r<numRay;r++) {
		Ray &ray = rays[r];
		ray.t = 1.0f;
		ray.contactFlag = false;
		ray.rayDir = ray.endPos - ray.startPos;
		
		// ブロードフェーズ探索する軸を選択
		Vector3 chkAxisVec = absPerElem(ray.rayDir);
		int chkAxis = 0;
		if(chkAxisVec[1] < chkAxisVec[0]) chkAxis = 1;
		if(chkAxisVec[2] < chkAxisVec[chkAxis]) chkAxis = 2;
		
		// レイのAABB作成
		VecInt3 rayStartL = worldVolume.worldToLocalPosition(ray.startPos);
		VecInt3 rayEndL = worldVolume.worldToLocalPosition(ray.endPos);
		VecInt3 rayMin = minPerElem(rayStartL,rayEndL);
		VecInt3 rayMax = maxPerElem(rayStartL,rayEndL);

		SortData rayAABB(0);
		setXMin(rayAABB,rayMin.getX());
		setXMax(rayAABB,rayMax.getX());
		setYMin(rayAABB,rayMin.getY());
		setYMax(rayAABB,rayMax.getY());
		setZMin(rayAABB,rayMin.getZ());
		setZMax(rayAABB,rayMax.getZ());

		// AABB探索開始
		int sign = ray.rayDir[chkAxis] < 0.0f ? -1 : 1; // 探索方向
		
		if(sign > 0) {
			traverseForward(ray,rayAABB,aabbArray[chkAxis],numAabb,chkAxis);
		}
		else {
			traverseBackward(ray,rayAABB,aabbArray[chkAxis],numAabb,chkAxis);
		}
	}

	pc.countEnd();
}

//J numStride単位でレイを処理します。同一ストライドに含まれるレイは基本的に近い属性
//J を持つようにあらかじめ設定してください。numStrideで割り切れない数のレイが入力された
//J 場合、余りのレイは処理されません。
void RayCast::ppuRayCastStride(Ray *rays,int numRay,int numStride,uint32_t flag)
{
	debugFlag = flag;
	PerfCounter pc;

	pc.countBegin("raycast");

	// -------------------------------------------------
	// ブロードフェーズ

	if(!broadphaseOk) {
		broadPhase(aabbArray,numAabb);
		broadphaseOk = true;
	}

	//PRINTF("::::: aabb :::::\n");
	//for(int i=0;i<numAabb;i++) {
	//	PRINTF("state %5u min %5u %5u %5u max %5u %5u %5u\n",
	//		StateId(aabbArray[0][i]),
	//		XMin(aabbArray[0][i]),YMin(aabbArray[0][i]),ZMin(aabbArray[0][i]),
	//		XMax(aabbArray[0][i]),YMax(aabbArray[0][i]),ZMax(aabbArray[0][i]));
	//}

	// -------------------------------------------------
	// レイキャスト

	for(int r=0;r<numRay;r+=numStride) {
		for(int i=0;i<numStride;i++) {
			Ray &ray = rays[r+i];
			ray.t = 1.0f;
			ray.contactFlag = false;
			ray.rayDir = ray.endPos - ray.startPos;
		}
		
		// ブロードフェーズ探索する軸を選択
		Vector3 chkAxisVec = absPerElem(rays[r].rayDir);
		int chkAxis = 0;
		if(chkAxisVec[1] < chkAxisVec[0]) chkAxis = 1;
		if(chkAxisVec[2] < chkAxisVec[chkAxis]) chkAxis = 2;

		int sign = rays[r].rayDir[chkAxis] < 0.0f ? -1 : 1; // 探索方向
		
		// レイのAABB作成
		SortData rayAABB(0);
		for(int i=0;i<numStride;i++) {
			Ray &ray = rays[r+i];

			VecInt3 rayStartL = worldVolume.worldToLocalPosition(ray.startPos);
			VecInt3 rayEndL = worldVolume.worldToLocalPosition(ray.endPos);
			VecInt3 rayMin = minPerElem(rayStartL,rayEndL);
			VecInt3 rayMax = maxPerElem(rayStartL,rayEndL);

			if(i==0) {
				setXMin(rayAABB,rayMin.getX());
				setXMax(rayAABB,rayMax.getX());
				setYMin(rayAABB,rayMin.getY());
				setYMax(rayAABB,rayMax.getY());
				setZMin(rayAABB,rayMin.getZ());
				setZMax(rayAABB,rayMax.getZ());
			}
			else {
				setXMin(rayAABB,PFX_MIN(XMin(rayAABB),rayMin.getX()));
				setXMax(rayAABB,PFX_MAX(XMax(rayAABB),rayMax.getX()));
				setYMin(rayAABB,PFX_MIN(YMin(rayAABB),rayMin.getY()));
				setYMax(rayAABB,PFX_MAX(YMax(rayAABB),rayMax.getY()));
				setZMin(rayAABB,PFX_MIN(ZMin(rayAABB),rayMin.getZ()));
				setZMax(rayAABB,PFX_MAX(ZMax(rayAABB),rayMax.getZ()));
			}
		}
		
		uint16_t aabbMin[3];
		uint16_t aabbMax[3];
		
		aabbMin[0] = XMin(rayAABB);
		aabbMax[0] = XMax(rayAABB);
		aabbMin[1] = YMin(rayAABB);
		aabbMax[1] = YMax(rayAABB);
		aabbMin[2] = ZMin(rayAABB);
		aabbMax[2] = ZMax(rayAABB);

		// AABB探索開始
		if(sign > 0) {
			traverseForwardStride(&rays[r],numStride,rayAABB,aabbArray[chkAxis],numAabb,chkAxis);
		}
		else {
			traverseBackwardStride(&rays[r],numStride,rayAABB,aabbArray[chkAxis],numAabb,chkAxis);
		}
	}

	pc.countEnd();
}

void RayCast::traverseForward(Ray &ray,SortData rayAABB,SortData *aabbArray,int numAabb,int chkAxis)
{
	for(int i=0;i<numAabb && XYZMax(rayAABB,chkAxis) >= XYZMin(aabbArray[i],chkAxis);i++) {
		if(XYZMax(aabbArray[i],chkAxis) < XYZMin(rayAABB,chkAxis)) {
			continue;
		}

		uint16_t stateIndex = StateId(aabbArray[i]);
		uint32_t contactFilterSelf = Self(aabbArray[i]);
		uint32_t contactFilterTarget = Target(aabbArray[i]);

		if(isCollidable(ray,stateIndex,contactFilterSelf,contactFilterTarget) && testAABB16(rayAABB,aabbArray[i])) {
			TrbState &state = states[stateIndex];
			Vector3 center(state.center[0],state.center[1],state.center[2]);
			Vector3 half(state.half[0],state.half[1],state.half[2]);
			float t_;
			if(rayIntersectAABBFast(half,center,ray.startPos,ray.rayDir,t_) && t_ < ray.t) {
				Transform3 transform(state.getOrientation(), state.getPosition());
				CollObject &coll = colls[BodyId(aabbArray[i])];
				if(rayIntersect(coll,transform,ray,ray.t)) {
					ray.contactFlag = true;
					ray.contactInstance = stateIndex;
				}
			}
		}
	}
}

void RayCast::traverseBackward(Ray &ray,SortData rayAABB,SortData *aabbArray,int numAabb,int chkAxis)
{
	(void) chkAxis;
	for(int i=numAabb-1;i>=0;i--) {
		uint16_t stateIndex = StateId(aabbArray[i]);
		uint32_t contactFilterSelf = Self(aabbArray[i]);
		uint32_t contactFilterTarget = Target(aabbArray[i]);
		
		if(isCollidable(ray,stateIndex,contactFilterSelf,contactFilterTarget) && testAABB16(rayAABB,aabbArray[i])) {
			TrbState &state = states[stateIndex];
			Vector3 center(state.center[0],state.center[1],state.center[2]);
			Vector3 half(state.half[0],state.half[1],state.half[2]);
			float t_;
			if(rayIntersectAABBFast(half,center,ray.startPos,ray.rayDir,t_) && t_ < ray.t) {
				Transform3 transform(state.getOrientation(), state.getPosition());
				CollObject &coll = colls[BodyId(aabbArray[i])];
				if(rayIntersect(coll,transform,ray,ray.t)) {
					ray.contactFlag = true;
					ray.contactInstance = stateIndex;
				}
			}
		}
	}
}

void RayCast::traverseForwardStride(Ray *rays,int numStride,SortData rayAABB,SortData *aabbArray,int numAabb,int chkAxis)
{
	// レイをブロードフェーズ座標系に正規化
	for(int i=0;i<numAabb && XYZMax(rayAABB,chkAxis) >= XYZMin(aabbArray[i],chkAxis);i++) {
		if(XYZMax(aabbArray[i],chkAxis) < XYZMin(rayAABB,chkAxis))
			continue;

		if(!testAABB16(rayAABB,aabbArray[i]))
			continue;

		uint16_t stateIndex = StateId(aabbArray[i]);
		uint32_t contactFilterSelf = Self(aabbArray[i]);
		uint32_t contactFilterTarget = Target(aabbArray[i]);

		TrbState &state = states[stateIndex];

		for(int r=0;r<numStride;r++) {
			Ray &ray = rays[r];
			if(isCollidable(ray,stateIndex,contactFilterSelf,contactFilterTarget)) {
				Vector3 center(state.center[0],state.center[1],state.center[2]);
				Vector3 half(state.half[0],state.half[1],state.half[2]);
				float t_;
				if(rayIntersectAABBFast(half,center,ray.startPos,ray.rayDir,t_) && t_ < ray.t) {
					Transform3 transform(state.getOrientation(), state.getPosition());
					CollObject &coll = colls[BodyId(aabbArray[i])];
					if(rayIntersect(coll,transform,ray,ray.t)) {
						ray.contactFlag = true;
						ray.contactInstance = stateIndex;
					}
				}
			}
		}
	}
}

void RayCast::traverseBackwardStride(Ray *rays,int numStride,SortData rayAABB,SortData *aabbArray,int numAabb,int chkAxis)
{
	(void) chkAxis;
	for(int i=numAabb-1;i>=0;i--) {
		if(!testAABB16(rayAABB,aabbArray[i]))
			continue;

		uint16_t stateIndex = StateId(aabbArray[i]);
		uint32_t contactFilterSelf = Self(aabbArray[i]);
		uint32_t contactFilterTarget = Target(aabbArray[i]);
		
		TrbState &state = states[stateIndex];
		
		for(int r=0;r<numStride;r++) {
			Ray &ray = rays[r];
			if(isCollidable(ray,stateIndex,contactFilterSelf,contactFilterTarget)) {
				Vector3 center(state.center[0],state.center[1],state.center[2]);
				Vector3 half(state.half[0],state.half[1],state.half[2]);
				float t_;
				if(rayIntersectAABBFast(half,center,ray.startPos,ray.rayDir,t_) && t_ < ray.t) {
					Transform3 transform(state.getOrientation(), state.getPosition());
					CollObject &coll = colls[BodyId(aabbArray[i])];
					if(rayIntersect(coll,transform,ray,ray.t)) {
						ray.contactFlag = true;
						ray.contactInstance = stateIndex;
					}
				}
			}
		}
	}
}



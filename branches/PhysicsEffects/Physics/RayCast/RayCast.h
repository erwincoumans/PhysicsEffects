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

#ifndef __RAYCAST_H__
#define __RAYCAST_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/Base/PerfCounter.h"
#include "Physics/Base/HeapManager.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"
#include "Physics/RigidBody/common/WorldVolume.h"
#include "Physics/RayCast/common/RayCastIO.h"
#include "Physics/RayCast/common/RayCastConfig.h"
#include "Physics/RayCast/common/RayIntersect.h"
#include "Physics/RayCast/common/Ray.h"

struct RayCastTaskMulti;

#define REPSILON 0.00001f

class RigidBodies;

///////////////////////////////////////////////////////////////////////////////
// RaycastProperty

struct RaycastProperty {
	uint32_t	maxInstances;			// 剛体インスタンスの最大数
	uint32_t	maxRayGroups;			// レイグループの最大数
	Vector3 	worldCenter;			// ワールドの中心
	Vector3		worldExtent;			// ワールドの広さ
	bool 		useRigidBodyWorldSize;	// 剛体のワールド情報を使うかどうか
	
	RaycastProperty() {
		maxInstances = 550;
		maxRayGroups = 1;
		worldExtent = Vector3(200.0f);
		worldCenter = Vector3(0.0f,90.0f,0.0f);
		useRigidBodyWorldSize = true;
	}
};

///////////////////////////////////////////////////////////////////////////////
// RayCast

class RayCast
{
private:
	HeapManager *mPool;

	// -------------------------------------------------------
	// Parameter

#ifndef WIN32
	RayCastTaskMulti* mRCTask;
	int mRCTaskID;
#endif

	// 剛体の配列
	TrbState *states;
	CollObject *colls;
	uint32_t numInstances;
	
	// ブロードフェーズ
	WorldVolume worldVolume;
	
	// 剛体ワールド
	RigidBodies *mRigidBodies;

	uint32_t debugFlag;

	// -------------------------------------------------------
	// Ray Cast

	RaycastProperty raycastProperty;

	bool broadphaseOk;
	
	uint32_t *nonContactFlag __attribute__ ((aligned(16))); // 衝突回避チェック用ビットフラグ
	uint32_t sizeContactTable;
	
	int numAabb;
	SortData *aabbArray[3] __attribute__ ((aligned(16)));	// AABBの各XYZ軸配列

	inline bool isCollidable(const Ray &ray,uint16_t rigidbodyIndex,uint32_t contactFilterSelf,uint32_t contactFilterTarget);

	void broadPhase(SortData *aabbArray[3],int &numAabb);
	void traverseForward(Ray &ray,SortData rayAABB,SortData *aabbArray,int numAabb,int chkAxis);
	void traverseBackward(Ray &ray,SortData rayAABB,SortData *aabbArray,int numAabb,int chkAxis);
	void traverseForwardStride(Ray *rays,int numStride,SortData rayAABB,SortData *aabbArray,int numAabb,int chkAxis);
	void traverseBackwardStride(Ray *rays,int numStride,SortData rayAABB,SortData *aabbArray,int numAabb,int chkAxis);


	// -------------------------------------------------------
	// Initialize

	void allocateBuffers();
	void deallocateBuffers();

	void setupWorldSize();

	RayCast() {}

public:

#ifndef WIN32
	RayCast(RayCastTaskMulti *RCTask,int taskID,HeapManager *pool);
#else
	RayCast(HeapManager *pool);
#endif

	virtual ~RayCast()
	{
		deallocateBuffers();
	}

	void reset();
	void setup() {}

	void printBufSize();
	uint32_t calcBufSize();
	
	// -------------------------------------------------------
	// Attach / Detach RigidBodies
	
	void attachRigidBodies(RigidBodies *rb);
	void detachRigidBodies();
	
	// -------------------------------------------------------
	// Ray Cast Setup

	void setupRayCast();
	void finalizeRayCast();

	// -------------------------------------------------------
	// Rat Cast Property
	
	// ※ レイキャスト設定を変更した後は必ずreset()を呼び出してください
	void setRaycastProperty(RaycastProperty &raycastProp) {raycastProperty = raycastProp;}
	
	void getWorldSize(Vector3 &center,Vector3 &extent);
	void setWorldSize(const Vector3 &center,const Vector3 &extent);

	// -------------------------------------------------------
	// Non Contact Body
	
	void appendNonContactPair(uint8_t rayGroup,uint16_t rigidbodyIndex);
	void removeNonContactPair(uint8_t rayGroup,uint16_t rigidbodyIndex);
	void clearNonContactFlag();

	// -------------------------------------------------------
	// PPU Ray Cast
	
	void ppuRayCast(Ray *rays,int numRay,uint32_t flag=0);
	void ppuRayCastStride(Ray *rays,int numRay,int numStride,uint32_t flag=0);
	

};

inline
bool RayCast::isCollidable(const Ray &ray,uint16_t rigidbodyIndex,uint32_t contactFilterSelf,uint32_t contactFilterTarget)
{
	uint32_t idx = ray.rayGroup * raycastProperty.maxRayGroups + rigidbodyIndex;
	uint32_t mask = 1L << (idx & 31);

	return (nonContactFlag[idx>>5] & mask) == 0 && 
		(ray.contactFilterSelf&contactFilterTarget) && (ray.contactFilterTarget&contactFilterSelf);
}

#endif

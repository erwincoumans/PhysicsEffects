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
#include "Physics/RigidBody/common/contactSolver.h"
#include "Physics/RigidBody/common/jointSolver.h"
#include "Physics/RigidBody/common/solverFuncTable.h"
#include "Physics/RigidBody/common/vec_utils.h"

#ifndef WIN32
RigidBodies::RigidBodies(RigidBodyTaskMulti *RBTask,int taskID,HeapManager *pool)
{
	mRBTask = RBTask;
	mRBTaskID = taskID;

#else
RigidBodies::RigidBodies(HeapManager *pool)
{
#endif

	forces = NULL;
	statesBuffer[0] = NULL;
	statesBuffer[1] = NULL;
	prevStates = NULL;
	bodies = NULL;
	collObjs = NULL;
	prims = NULL;
	joints = NULL;
	springs = NULL;
	contactPairs = NULL;
	sortedContactPairs = NULL;
	nonContactPair = NULL;
	bodiesPool = NULL;
	collsPool = NULL;
	primsPool = NULL;
	statesPool = NULL;
	jointsPool = NULL;
	springsPool = NULL;
	userData = NULL;

	// プールメモリ
	mPool = pool;
}

void RigidBodies::printWorld()
{
	#define PRINTNUM(name,val,maxVal) PRINTF(name " : %d/%d\n",val,maxVal);
	
	PRINTF(" --- World Info ---\n");
	PRINTNUM("number of bodies",numBodies,worldProperty.maxDynBodies);
	PRINTNUM("number of instances",numInstances,worldProperty.maxInstances);
	PRINTNUM("number of joints",numJoints,worldProperty.maxJoints);
	PRINTNUM("number of springs",numSprings,worldProperty.maxSprings);
	
	#undef PRINTNUM
}

void RigidBodies::printBufSize()
{
	#define PRINTBUF(val,type,count) \
		PRINTF(#val " : sizeof(" #type ") %d bytes * %d = %d bytes\n",sizeof(type),count,sizeof(type)*count);\
		allocatedBytes+=sizeof(type)*count;

	{
		int allocatedBytes = 0;
		PRINTF(" --- RigidBody XDR buffers ---\n");
		PRINTBUF(forces,Forces,worldProperty.maxInstances);
		PRINTBUF(statesBuffer[0],TrbState,worldProperty.maxInstances);
		PRINTBUF(statesBuffer[1],TrbState,worldProperty.maxInstances);
		PRINTBUF(prevStates,TrbState,worldProperty.maxInstances);
		PRINTBUF(bodies,TrbDynBody,worldProperty.maxDynBodies);
		PRINTBUF(collObjs,CollObject,worldProperty.maxDynBodies);
		PRINTBUF(prims,CollPrim,worldProperty.maxPrimitives);
		PRINTBUF(joints,Joint,worldProperty.maxJoints);
		PRINTBUF(springs,Spring,worldProperty.maxSprings);
		PRINTBUF(contactPairs,ContactPair,worldProperty.maxContactPairs);
		PRINTBUF(sortedContactPairs,SortData,worldProperty.maxContactPairs);
		
		PRINTBUF(nonContactPair,uint32_t,sizeContactTable);
		PRINTBUF(userData,uint32_t,worldProperty.maxInstances);

		PRINTBUF(bodiesPool,uint16_t,worldProperty.maxDynBodies);
		PRINTBUF(collsPool,uint16_t,worldProperty.maxDynBodies);
		PRINTBUF(primsPool,uint16_t,worldProperty.maxPrimitives);
		PRINTBUF(statesPool,uint16_t,worldProperty.maxInstances);
		PRINTBUF(jointsPool,uint16_t,worldProperty.maxJoints);
		PRINTBUF(springsPool,uint16_t,worldProperty.maxSprings);

		PRINTF("total : %d bytes\n",allocatedBytes);
	}
	
	{
		PRINTF(" ---RigidBody ETC buffers ---\n");
		PRINTF("sizeof(HeightField) %d bytes , sizeof(HeightFieldBlock) %d bytes\n",
			sizeof(HeightField),sizeof(HeightFieldBlock));
		PRINTF("sizeof(LargeTriMesh) %d bytes , sizeof(TriMesh) %d bytes\n",
			sizeof(LargeTriMesh),sizeof(TriMesh));
	}
	
	#undef PRINTBUF
}

uint32_t RigidBodies::calcBufSize()
{
	#define CALCBUF(type,count) (sizeof(type)*count)

	int bufSize = 0;
	int maxPair = (worldProperty.maxInstances*(worldProperty.maxInstances-1)/2);
	int contactTable = PFX_ALIGN128((maxPair+31)/32,sizeof(uint32_t));

	bufSize += CALCBUF(Forces,worldProperty.maxInstances);
	bufSize += CALCBUF(TrbState,worldProperty.maxInstances); // TrbState Buff0
	bufSize += CALCBUF(TrbState,worldProperty.maxInstances); // TrbState Buff0
	bufSize += CALCBUF(TrbState,worldProperty.maxInstances); // TrbState Prev
	bufSize += CALCBUF(TrbDynBody,worldProperty.maxDynBodies);
	bufSize += CALCBUF(CollObject,worldProperty.maxDynBodies);
	bufSize += CALCBUF(CollPrim,worldProperty.maxPrimitives);
	bufSize += CALCBUF(Joint,worldProperty.maxJoints);
	bufSize += CALCBUF(ContactPair,worldProperty.maxContactPairs);
	bufSize += CALCBUF(SortData,worldProperty.maxContactPairs);
	bufSize += CALCBUF(uint32_t,contactTable); // contact table
	bufSize += CALCBUF(uint32_t,worldProperty.maxInstances);	// userData
	bufSize += CALCBUF(uint16_t,worldProperty.maxDynBodies);	// body pool
	bufSize += CALCBUF(uint16_t,worldProperty.maxDynBodies);	// coll pool
	bufSize += CALCBUF(uint16_t,worldProperty.maxPrimitives);	// prim pool
	bufSize += CALCBUF(uint16_t,worldProperty.maxInstances);	// state pool
	bufSize += CALCBUF(uint16_t,worldProperty.maxInstances);	// joint pool

	return bufSize;

	#undef CALCBUF
}

void RigidBodies::allocateBuffers()
{
	#define ALLOCATE(align,size) mPool->allocate(size,align==128?HeapManager::ALIGN128:HeapManager::ALIGN16);

	int maxPair = (worldProperty.maxInstances*(worldProperty.maxInstances-1)/2);
	sizeContactTable = PFX_ALIGN128((maxPair+31)/32,sizeof(uint32_t));

	printBufSize();

	forces = (Forces*)ALLOCATE(128,sizeof(Forces)*worldProperty.maxInstances);
	statesBuffer[0] = (TrbState*)ALLOCATE(128,sizeof(TrbState)*worldProperty.maxInstances);
	statesBuffer[1] = (TrbState*)ALLOCATE(128,sizeof(TrbState)*worldProperty.maxInstances);
	prevStates = (TrbState*)ALLOCATE(128,sizeof(TrbState)*worldProperty.maxInstances);
	bodies = (TrbDynBody*)ALLOCATE(128,sizeof(TrbDynBody)*worldProperty.maxDynBodies);
	collObjs = (CollObject*)ALLOCATE(128,sizeof(CollObject)*worldProperty.maxDynBodies);
	prims = (CollPrim*)ALLOCATE(128,sizeof(CollPrim)*worldProperty.maxPrimitives);
	joints = (Joint*)ALLOCATE(128,sizeof(Joint)*worldProperty.maxJoints);
	springs = (Spring*)ALLOCATE(128,sizeof(Spring)*worldProperty.maxSprings);
	contactPairs = (ContactPair*)ALLOCATE(128,sizeof(ContactPair)*worldProperty.maxContactPairs);
	sortedContactPairs = (SortData*)ALLOCATE(128,sizeof(SortData)*worldProperty.maxContactPairs);

	nonContactPair = (uint32_t*)ALLOCATE(128,sizeof(uint32_t)*sizeContactTable);
	userData = (uint32_t*)ALLOCATE(128,sizeof(uint32_t)*worldProperty.maxInstances);

	bodiesPool  = (uint16_t*)ALLOCATE(16,sizeof(uint16_t)*worldProperty.maxDynBodies);
	collsPool = (uint16_t*)ALLOCATE(16,sizeof(uint16_t)*worldProperty.maxDynBodies);
	primsPool = (uint16_t*)ALLOCATE(16,sizeof(uint16_t)*worldProperty.maxPrimitives);
	statesPool  = (uint16_t*)ALLOCATE(16,sizeof(uint16_t)*worldProperty.maxInstances);
	jointsPool  = (uint16_t*)ALLOCATE(16,sizeof(uint16_t)*worldProperty.maxJoints);
	springsPool = (uint16_t*)ALLOCATE(16,sizeof(uint16_t)*worldProperty.maxSprings);

	#undef ALLOCATE
}

void RigidBodies::deallocateBuffers()
{
	#define DEALLOCATE(ptr) if(ptr) {mPool->deallocate(ptr);ptr=NULL;}

	DEALLOCATE(springsPool);
	DEALLOCATE(jointsPool);
	DEALLOCATE(statesPool);
	DEALLOCATE(primsPool);
	DEALLOCATE(collsPool);
	DEALLOCATE(bodiesPool);
	DEALLOCATE(userData);
	DEALLOCATE(nonContactPair);
	DEALLOCATE(sortedContactPairs);
	DEALLOCATE(contactPairs);
	DEALLOCATE(springs);
	DEALLOCATE(joints);
	DEALLOCATE(prims);
	DEALLOCATE(collObjs);
	DEALLOCATE(bodies);
	DEALLOCATE(prevStates);
	DEALLOCATE(statesBuffer[1]);
	DEALLOCATE(statesBuffer[0]);
	DEALLOCATE(forces);

	#undef DEALLOCATE
}

void RigidBodies::setupWorldSize()
{
	worldVolume.setWorldSize(worldProperty.worldCenter,worldProperty.worldExtent);
}

void RigidBodies::getWorldSize(Vector3 &center,Vector3 &extent)
{
	center = worldProperty.worldCenter;
	extent = worldProperty.worldExtent;
}

void RigidBodies::setWorldSize(const Vector3 &center,const Vector3 &extent)
{
	worldProperty.worldCenter = center;
	worldProperty.worldExtent = extent;
	setupWorldSize();
}

void RigidBodies::printContactPairs()
{
	if(numContactPairs > 0)
		PRINTF("***** print contact pairs *****\n");

	for(uint32_t i=0;i<numContactPairs;i++) {
		PRINTF("%5d key %7d pair %5d S %d %d B %d %d ",i,Key(sortedContactPairs[i]),Pair(sortedContactPairs[i]),
			StateA(sortedContactPairs[i]),StateB(sortedContactPairs[i]),
			BodyA(sortedContactPairs[i]),BodyB(sortedContactPairs[i]));
		ContactPair &c = contactPairs[Pair(sortedContactPairs[i])];
		PRINTF("d %d numCp %d",c.duration,c.numContacts);
		for(int j=0;j<c.numContacts;j++) {
			ContactPoint &cp = c.contactPoints[j];
			PRINTF(" (%d d %d aI %.2f aF %.2f %.2f)",j,cp.duration,cp.constraints[0].accumImpulse,cp.constraints[1].accumImpulse,cp.constraints[2].accumImpulse);
		}
		PRINTF("\n");
	}
}

// -------------------------------------------------------
// Construct Rigid Body

int	RigidBodies::createCollPrim()
{
	uint32_t newIdx;

	if(numPrimsPool > 0) {
		newIdx = primsPool[--numPrimsPool];
	}
	else if(numPrims < worldProperty.maxPrimitives){
		newIdx = numPrims++;
	}
	else {
		return -1;
	}

	prims[newIdx].reset();

	return (int)newIdx;
}

bool RigidBodies::deleteCollPrim(int primId)
{
	uint32_t delIdx = primId;
	
	if(delIdx >= worldProperty.maxPrimitives)
		return false;

	for(uint32_t i=0;i<numPrimsPool;i++) {
		if(primsPool[i] == delIdx) {
			return false;
		}
	}

	primsPool[numPrimsPool++] = delIdx;

	return true;
}

CollObject*	RigidBodies::createCollObject()
{
	uint32_t newIdx;

	if(numCollsPool > 0) {
		newIdx = collsPool[--numCollsPool];
	}
	else if(numCollObjs < worldProperty.maxDynBodies){
		newIdx = numCollObjs++;
	}
	else {
		return NULL;
	}

	collObjs[newIdx].clear();
	collObjs[newIdx].mPrimBase = prims;
	collObjs[newIdx].mMaxPrims = 1;
	return &collObjs[newIdx];
}

CollObject*	RigidBodies::createCollObject(int n)
{
	if(n < 2) return createCollObject();
	
	uint32_t newIdx;

	if(numCollsPool > 0) {
		newIdx = collsPool[--numCollsPool];
	}
	else if(numCollObjs < worldProperty.maxDynBodies){
		newIdx = numCollObjs++;
	}
	else {
		return NULL;
	}

	collObjs[newIdx].clear();
	collObjs[newIdx].mPrimBase = prims;
	collObjs[newIdx].mMaxPrims = n;
	for(int i=0;i<n-1;i++) {
		int newPrimId = createCollPrim();
		PFX_ASSERT(newPrimId >= 0);
		collObjs[newIdx].mPrimIds[i] = (uint16_t)newPrimId;
	}
	return &collObjs[newIdx];
}

bool RigidBodies::deleteCollObject(CollObject *coll)
{
	uint32_t delIdx = coll - collObjs;
	
	if(delIdx >= worldProperty.maxDynBodies)
		return false;

	for(uint16_t i=0;i<numCollsPool;i++) {
		if(collsPool[i] == delIdx)
			return false;
	}

	PFX_ASSERT(coll->mPrimBase == prims);

	for(int i=0;i<collObjs[delIdx].mNumPrims-1;i++) {
		deleteCollPrim(collObjs[delIdx].mPrimIds[i]);
	}

	collsPool[numCollsPool++] = delIdx;

	return true;
}

TrbDynBody* RigidBodies::createRigidBody(const RigidBodyProperty &param)
{
	if(!param.collObject) return NULL;
	
	uint32_t newIdx;
	if(numBodiesPool > 0) {
		newIdx = bodiesPool[--numBodiesPool];
	}
	else if(numBodies < worldProperty.maxDynBodies){
		newIdx = numBodies++;
	}
	else {
		return NULL;
	}

	bodies[newIdx].setMass(param.mass);
	bodies[newIdx].setFriction(param.friction);
	bodies[newIdx].setElasticity(param.restitution);
	bodies[newIdx].setBodyInertia(param.inertia);
	bodies[newIdx].setCollObject(param.collObject);

	return &bodies[newIdx];
}

bool RigidBodies::deleteRigidBody(TrbDynBody *rigidbody)
{
	uint32_t delIdx = rigidbody - bodies;
	
	if(delIdx >= worldProperty.maxDynBodies)
		return false;

	for(uint16_t i=0;i<numBodiesPool;i++) {
		if(bodiesPool[i] == delIdx)
			return false;
	}

	bodiesPool[numBodiesPool++] = delIdx;

	return true;
}

int RigidBodies::createInstance(const InstanceProperty &param)
{
	if(!param.rigidBody) return -1;

	uint32_t newIdx;
	if(numStatesPool > 0) {
		newIdx = statesPool[--numStatesPool];
	}
	else if(numInstances < worldProperty.maxInstances){
		newIdx = numInstances++;
	}
	else {
		return -1;
	}

	if(param.rigidBody->getMass() <= 0.0f && param.moveType != MoveTypeKeyframe)
		statesBuffer[readBuffer][newIdx].setMoveType(MoveTypeFixed);
	else
		statesBuffer[readBuffer][newIdx].setMoveType(param.moveType);

	if(param.sleeping == 1) statesBuffer[readBuffer][newIdx].sleep();

	statesBuffer[readBuffer][newIdx].setContactFilterSelf(param.contactFilterSelf);
	statesBuffer[readBuffer][newIdx].setContactFilterTarget(param.contactFilterTarget);
	statesBuffer[readBuffer][newIdx].linearDamping = param.linearDamping;
	statesBuffer[readBuffer][newIdx].angularDamping = param.angularDamping;
	statesBuffer[readBuffer][newIdx].setPosition(param.position);
	statesBuffer[readBuffer][newIdx].setOrientation(param.orientation);
	statesBuffer[readBuffer][newIdx].setLinearVelocity(param.velocity);
	statesBuffer[readBuffer][newIdx].setAngularVelocity(param.angularVelocity);
	statesBuffer[readBuffer][newIdx].trbBodyIdx = ((uint32_t)param.rigidBody - (uint32_t)bodies) / sizeof(TrbDynBody);
	statesBuffer[readBuffer][newIdx].deleted = 0;
	statesBuffer[readBuffer][newIdx].useSleep = 1;
	statesBuffer[readBuffer][newIdx].useCcd = 0;
	statesBuffer[readBuffer][newIdx].useContactCallback = 0;
	statesBuffer[readBuffer][newIdx].useSleepCallback = 0;
	statesBuffer[readBuffer][newIdx].setDeltaLinearVelocity(Vector3(0.0f));
	statesBuffer[readBuffer][newIdx].setDeltaAngularVelocity(Vector3(0.0f));

	return (int)newIdx;
}

bool RigidBodies::deleteInstance(int stateIndex)
{
	if(stateIndex < 0 || stateIndex >= (int)worldProperty.maxInstances)
		return false;

	for(uint16_t i=0;i<numStatesPool;i++) {
		if(statesPool[i] == stateIndex)
			return false;
	}

	statesPool[numStatesPool++] = stateIndex;
	
	// 消去した剛体をブロードフェーズ領域外に配置
	TrbState *delState = getState(stateIndex);
	delState->setMoveType(MoveTypeFixed);
	delState->deleted = 1;
	Vector3 half(delState->half[0],delState->half[1],delState->half[2]);
	setPosition(stateIndex,worldVolume.origin + worldVolume.extent + half);
	return true;
}

int RigidBodies::cloneInstance(uint32_t instance)
{
	if(instance >= numInstances) return -1;

	uint32_t newIdx;
	if(numStatesPool > 0) {
		newIdx = statesPool[--numStatesPool];
	}
	else if(numInstances < worldProperty.maxInstances){
		newIdx = numInstances++;
	}
	else {
		return -1;
	}

	statesBuffer[readBuffer][newIdx] = statesBuffer[readBuffer][instance];
	statesBuffer[readBuffer][newIdx].trbBodyIdx = statesBuffer[readBuffer][instance].trbBodyIdx;

	return (int)newIdx;
}

int RigidBodies::createSpring(float length,float ks,float kd,uint16_t stateIndexA,uint16_t stateIndexB,const Vector3 &worldAnchor)
{
	Matrix3 rotA = transpose(Matrix3(statesBuffer[readBuffer][stateIndexA].getOrientation()));
	Matrix3 rotB = transpose(Matrix3(statesBuffer[readBuffer][stateIndexB].getOrientation()));

	return createSpring(
		length,ks,kd,
		stateIndexA,stateIndexB,
		rotA * (worldAnchor - statesBuffer[readBuffer][stateIndexA].getPosition()),
		rotB * (worldAnchor - statesBuffer[readBuffer][stateIndexB].getPosition()));
}

int RigidBodies::createSpring(float length,float ks,float kd,uint16_t stateIndexA,uint16_t stateIndexB,const Vector3 &localAnchorA,const Vector3 &localAnchorB)
{
	uint32_t newIdx;
	if(numSpringsPool > 0) {
		newIdx = springsPool[--numSpringsPool];
	}
	else if(numSprings < worldProperty.maxSprings){
		newIdx = numSprings++;
	}
	else {
		return -1;
	}
	
	springs[newIdx].length = length;
	springs[newIdx].ks = ks;
	springs[newIdx].kd = kd;
	springs[newIdx].active = true;
	springs[newIdx].stateIndexA = stateIndexA;
	springs[newIdx].stateIndexB = stateIndexB;
	springs[newIdx].anchorA = localAnchorA;
	springs[newIdx].anchorB = localAnchorB;

	return (int)newIdx;
}

bool RigidBodies::deleteSpring(int springIndex)
{
	if(springIndex < 0 || springIndex >= (int)worldProperty.maxSprings)
		return false;

	for(uint16_t i=0;i<numSpringsPool;i++) {
		if(springsPool[i] == springIndex)
			return false;
	}

	springsPool[numSpringsPool++] = springIndex;
	springs[springIndex].active = false;

	return true;
}

int RigidBodies::createJoint()
{
	uint32_t newIdx;
	if(numJointsPool > 0) {
		newIdx = jointsPool[--numJointsPool];
	}
	else if(numJoints < worldProperty.maxJoints){
		newIdx = numJoints++;
	}
	else {
		return -1;
	}
	
	return (int)newIdx;
}

int RigidBodies::createJoint(const JointProperty &jointParam)
{
	PFX_ASSERT(jointParam.jointType < JointTypeCount);

	int jointIdx = createJoint();
	PFX_ASSERT(jointIdx >= 0);
	
	// ジョイントを初期化
	Joint &newJoint = joints[jointIdx];
	newJoint.reset();

	// ジョイントパラメータの適応
	newJoint.jointType = jointParam.jointType;
	newJoint.linearDamping = jointParam.linearDamping;
	newJoint.angularDamping = jointParam.angularDamping;
	newJoint.maxLinearImpulse = jointParam.maxLinearImpulse;
	newJoint.maxAngularImpulse = jointParam.maxAngularImpulse;
	newJoint.linearImpulseWeight = jointParam.linearImpulseWeight;
	newJoint.angularImpulseWeight = jointParam.angularImpulseWeight;
	newJoint.linearBias = jointParam.linearBias;
	newJoint.angularBias = jointParam.angularBias;
	newJoint.targetFrame = Matrix3::identity();
	newJoint.stateIndexA = jointParam.parentBody;
	newJoint.stateIndexB = jointParam.childBody;
	if(jointParam.breakableLimit > 0.0f) {
		newJoint.breakableLimit = jointParam.breakableLimit;
		newJoint.enableBreakable();
	}

	for(int i=0;i<6;i++) {
		if(jointParam.warmStarting[i])
			newJoint.enableWarmStarting(i);
		else
			newJoint.disableWarmStarting(i);
	}

	// ジョイントタイプ毎の特性
	switch(jointParam.jointType) {
		case JointTypeBall:
		newJoint.setLock(0);
		newJoint.setLock(1);
		newJoint.setLock(2);
		newJoint.setFree(3);
		newJoint.setFree(4);
		newJoint.setFree(5);
		break;

		case JointTypeChain:
		newJoint.setLock(0);
		newJoint.setLock(1);
		newJoint.setLock(2);
		newJoint.setLimit(3);
		newJoint.setLimit(4);
		newJoint.setFree(5);
		if(jointParam.lowerLimit1 < jointParam.upperLimit1) {
			newJoint.lowerLimit[3] = jointParam.lowerLimit1;
			newJoint.upperLimit[3] = jointParam.upperLimit1;
		}
		else {
			newJoint.lowerLimit[3] = -0.26f;
			newJoint.upperLimit[3] =  0.26f;
		}
		if(jointParam.lowerLimit2 < jointParam.upperLimit2) {
			newJoint.lowerLimit[4] = jointParam.lowerLimit2;
			newJoint.upperLimit[4] = jointParam.upperLimit2;
		}
		else {
			newJoint.lowerLimit[4] =  0.0f;
			newJoint.upperLimit[4] =  0.7f;
		}
		break;
		
		case JointTypeSlider:
		newJoint.setLock(1);
		newJoint.setLock(2);
		newJoint.setLock(3);
		newJoint.setLock(4);
		newJoint.setLock(5);
		if(jointParam.lowerLimit1 < jointParam.upperLimit1) {
			newJoint.setLimit(0);
			newJoint.lowerLimit[0] = jointParam.lowerLimit1;
			newJoint.upperLimit[0] = jointParam.upperLimit1;
		}
		else {
			newJoint.setFree(0);
		}
		break;
		
		case JointTypeHinge:
		newJoint.setLock(0);
		newJoint.setLock(1);
		newJoint.setLock(2);
		newJoint.setLock(4);
		newJoint.setLock(5);
		if(jointParam.lowerLimit1 < jointParam.upperLimit1) {
			newJoint.setLimit(3);
			newJoint.lowerLimit[3] = jointParam.lowerLimit1;
			newJoint.upperLimit[3] = jointParam.upperLimit1;
		}
		else {
			newJoint.setFree(3);
		}
		break;
		
		case JointTypeFix:
		case JointTypeAnimation:
		newJoint.setLock(0);
		newJoint.setLock(1);
		newJoint.setLock(2);
		newJoint.setLock(3);
		newJoint.setLock(4);
		newJoint.setLock(5);
		break;

		case JointTypeUniversal:
		newJoint.setLock(0);
		newJoint.setLock(1);
		newJoint.setLock(2);
		newJoint.setLock(3);
		newJoint.setLimit(4);
		newJoint.setLimit(5);
		if(jointParam.lowerLimit1 < jointParam.upperLimit1) {
			newJoint.lowerLimit[4] = jointParam.lowerLimit1;
			newJoint.upperLimit[4] = jointParam.upperLimit1;
		}
		else {
			newJoint.lowerLimit[4] = -0.5f;
			newJoint.upperLimit[4] =  0.5f;
		}
		if(jointParam.lowerLimit2 < jointParam.upperLimit2) {
			newJoint.lowerLimit[5] = jointParam.lowerLimit2;
			newJoint.upperLimit[5] = jointParam.upperLimit2;
		}
		else {
			newJoint.lowerLimit[5] =  -0.7f;
			newJoint.upperLimit[5] =   0.7f;
		}
		break;

		case JointTypeDistance:
		newJoint.lowerLimit[0] = jointParam.distance;
		newJoint.upperLimit[0] = jointParam.distance;
		newJoint.setLimit(0);
		newJoint.setLock(1);
		newJoint.setLock(2);
		newJoint.setFree(3);
		newJoint.setFree(4);
		newJoint.setFree(5);
		break;
	}
	
	// ジョイントのフレームを作成
	if(jointParam.parentBody < numInstances && jointParam.childBody < numInstances && 
	   jointParam.parentBody != jointParam.childBody) {
		TrbState &stateA = statesBuffer[readBuffer][jointParam.parentBody];
		TrbState &stateB = statesBuffer[readBuffer][jointParam.childBody];

		Matrix3 rotA = transpose(Matrix3(stateA.getOrientation()));
		Matrix3 rotB = transpose(Matrix3(stateB.getOrientation()));

		Vector3 axisInA = rotA * normalize(jointParam.axis);
		Vector3 axisInB = rotB * normalize(jointParam.axis);

		newJoint.anchorA = rotA * (jointParam.anchor - stateA.getPosition());
		newJoint.anchorB = rotB * (jointParam.anchor - stateB.getPosition());

		Vector3 axis1, axis2;

		getPlaneSpace(axisInA, axis1, axis2 );
		newJoint.frameA = Matrix3(axisInA, axis1, axis2);
		newJoint.frameB = rotB * Matrix3(stateA.getOrientation()) * newJoint.frameA;
	}
	else {
		newJoint.anchorA = Vector3(1.0f,0.0f,0.0f);
		newJoint.anchorB = Vector3(1.0f,0.0f,0.0f);
		newJoint.frameA = Matrix3::identity();
		newJoint.frameB = Matrix3::identity();
		newJoint.disableActive();
	}

	return (int)jointIdx;
}

bool RigidBodies::deleteJoint(int jointIndex)
{
	if(jointIndex < 0 || jointIndex >= (int)worldProperty.maxJoints)
		return false;

	for(uint16_t i=0;i<numJointsPool;i++) {
		if(jointsPool[i] == jointIndex)
			return false;
	}

	jointsPool[numJointsPool++] = jointIndex;
	joints[jointIndex].disableActive();

	return true;
}

// -------------------------------------------------------
// ContactPair Table

void RigidBodies::appendNonContactPair(uint16_t stateIndexA,uint16_t stateIndexB)
{
	PFX_ASSERT(stateIndexA != stateIndexB);
	PFX_ASSERT(stateIndexA < worldProperty.maxInstances);
	PFX_ASSERT(stateIndexB < worldProperty.maxInstances);
	
	uint32_t minIdx = stateIndexA>stateIndexB?stateIndexB:stateIndexA;
	uint32_t maxIdx = stateIndexA>stateIndexB?stateIndexA:stateIndexB;
	
	uint32_t idx = maxIdx * (maxIdx - 1) / 2 + minIdx;
	PFX_ASSERT((idx>>5) < sizeContactTable);

	nonContactPair[idx>>5] |= 1L << (idx & 31);
}

void RigidBodies::removeNonContactPair(uint16_t stateIndexA,uint16_t stateIndexB)
{
	PFX_ASSERT(stateIndexA != stateIndexB);
	PFX_ASSERT(stateIndexA < worldProperty.maxInstances);
	PFX_ASSERT(stateIndexB < worldProperty.maxInstances);
	
	uint32_t minIdx = stateIndexA>stateIndexB?stateIndexB:stateIndexA;
	uint32_t maxIdx = stateIndexA>stateIndexB?stateIndexA:stateIndexB;
	
	uint32_t idx = maxIdx * (maxIdx - 1) / 2 + minIdx;
	PFX_ASSERT((idx>>5) < sizeContactTable);

	nonContactPair[idx>>5] = nonContactPair[idx>>5] & ~(1L << (idx & 31));
}

bool RigidBodies::checkNonContactPair(uint16_t stateIndexA,uint16_t stateIndexB)
{
	return isCollidablePair(stateIndexA,stateIndexB);
}

// -------------------------------------------------------

void
RigidBodies::reset()
{
	// バッファの確保
	deallocateBuffers();
	allocateBuffers();

	clearNonContactPair();

	memset(userData,0,sizeof(uint32_t)*worldProperty.maxInstances);

	numInstances = 0;
	numBodies = 0;
	numCollObjs = 0;
	numPrims = 0;
	numJoints = 0;
	numSprings = 0;
	writeBuffer = 0;
	readBuffer = 1;

	states = statesBuffer[writeBuffer];
	
	// ブロードフェーズのサイズをセット
	setupWorldSize();
	
	// ソートコンタクトの初期化
	for(uint32_t i=0;i<worldProperty.maxContactPairs;i++) {
		setPair(sortedContactPairs[i],i);
	}
	numContactPairs = 0;

	// 再利用プール初期化
	numBodiesPool = 0;
	numCollsPool = 0;
	numPrimsPool = 0;
	numStatesPool = 0;
	numJointsPool = 0;
	numSpringsPool = 0;

	worldSleepCount = 0;

	debugFlag = 0;
}

void
RigidBodies::setup()
{
//	DPRINT("RigidBodies::setup in\n");

	for(uint32_t i=0;i<numInstances;i++) {
		setupRigidBody(i);
	}

	printWorld();

//	DPRINT("RigidBodies::setup out\n");
}

void
RigidBodies::setupRigidBody(uint16_t stateIndex)
{
	CollObject *coll = bodies[statesBuffer[readBuffer][stateIndex].trbBodyIdx].getCollObject();
	forces[stateIndex].force = forces[stateIndex].torque = Vector3(0.0f);
	statesBuffer[readBuffer][stateIndex].setAuxils(coll->getCenter(),coll->getHalf());
	statesBuffer[writeBuffer][stateIndex].setAuxils(coll->getCenter(),coll->getHalf());

	statesBuffer[readBuffer][stateIndex].sleepCount = 0;

	prevStates[stateIndex] = statesBuffer[writeBuffer][stateIndex] = statesBuffer[readBuffer][stateIndex];
}

void
RigidBodies::setupSimulate()
{
	// スプリングを外力として与える
	applySpring();
}

void
RigidBodies::finalizeSimulate()
{
	// 外力クリア
	memset(forces,0,sizeof(Forces)*numInstances);

	// バッファスワップ
	writeBuffer = 1-writeBuffer;
	readBuffer = 1-readBuffer;
}

// -------------------------------------------------------
// Integrate

template <typename T>
inline T RungeKutta(const T &deriv,float dt)
{
	T	k0, k1, k2, k3;
	k0 = deriv * dt;
	k1 = (deriv + k0 * 0.5f) * dt;
	k2 = (deriv + k1 * 0.5f) * dt;
	k3 = (deriv + k2) * dt;
	return (k0 + k1*2.0f + k2*2.0f + k3) / 6.0f;
}

void
RigidBodies::integrate(float timeStep,bool lastSubStep)
{
	for(uint16_t i = 0; i < numInstances; i++) {
		TrbState &state = states[i];
		prevStates[i] = state;

		if( state.isAsleep() || state.getMoveTypeBits() & ~MOVE_TYPE_CAN_SLEEP ) { 
			continue;
		}
		
		TrbDynBody &trbBody = bodies[state.trbBodyIdx];

		if(state.getMoveType() == MoveTypeKeyframe) {
			Quat derivQ = Quat(state.getAngularVelocity(),0) * state.getOrientation() * 0.5f;
			state.setPosition(state.getPosition()+state.getLinearVelocity() * timeStep);
			state.setOrientation(normalize(state.getOrientation() + derivQ * timeStep));
			if(lastSubStep) {
				state.setLinearVelocity(Vector3(0.0f));
				state.setAngularVelocity(Vector3(0.0f));
			}
		}
		else {
			Matrix3 fR(state.getOrientation());
			Matrix3 fI = fR * trbBody.getBodyInertia() * transpose(fR);
			Matrix3 fIInv = fR * trbBody.getBodyInertiaInv() * transpose(fR);
			
			// compute derivs
			Vector3 totalForce = forces[i].force + worldProperty.gravity * trbBody.getMass();
			Vector3 totalTorque = forces[i].torque;
			Vector3 dx = state.getLinearVelocity();
			Quat dq = Quat(state.getAngularVelocity(),0) * state.getOrientation() * 0.5f;
			Vector3 dv = totalForce * trbBody.getMassInv();
			Vector3 dw = fIInv * (cross(fI * state.getAngularVelocity(), state.getAngularVelocity()) + totalTorque);

			Vector3 nx = state.getPosition();
			Quat    nq = state.getOrientation();
			Vector3 nv = state.getLinearVelocity();
			Vector3 nw = state.getAngularVelocity();

#ifdef ODE_EULER
			nx += dx * timeStep;
			nq += dq * timeStep;
			nq = normalize(nq);
			nv += dv * timeStep;
			nw += dw * timeStep;
#else
			nx += RungeKutta(dx,timeStep);
			nq += RungeKutta(dq,timeStep);
			nq = normalize(nq);
			nv += RungeKutta(dv,timeStep);
			nw += RungeKutta(dw,timeStep);
#endif

			// apply damping
			nv *= state.linearDamping;
			nw *= state.angularDamping;
			
			state.setPosition(nx);
			state.setOrientation(nq);
			state.setLinearVelocity(nv);
			state.setAngularVelocity(nw);
		}

		// AABB更新
		if(worldProperty.ccdEnable && state.getUseCcd()) {
			// 速度方向にAABBを伸ばす
			state.setAuxilsCcd(trbBody.getCollObject()->getCenter(),trbBody.getCollObject()->getHalf(),timeStep);
		}
		else {
			state.setAuxils(trbBody.getCollObject()->getCenter(),trbBody.getCollObject()->getHalf());
		}
	}
}

// -------------------------------------------------------
// PPU Collision & Response

void RigidBodies::preResponse(const float timeStep)
{
	ContactSolverConfig contactSolverCfg;
	contactSolverCfg.timeStep = timeStep;
	contactSolverCfg.separateBias = worldProperty.separateBias;
	contactSolverCfg.deformMeshEnable = worldProperty.deformMeshEnable;

	for(uint32_t i=0;i<numContactPairs;i++) {
		ContactPair *contactPair = getContactPair(i);
		TrbState &stateA = states[contactPair->stateIndex[0]];
		TrbState &stateB = states[contactPair->stateIndex[1]];
		TrbDynBody &bodyA = bodies[stateA.trbBodyIdx];
		TrbDynBody &bodyB = bodies[stateB.trbBodyIdx];
		funcTbl_preResponse[stateA.moveType+stateA.sleeping][stateB.moveType+stateB.sleeping](*contactPair,stateA,bodyA,stateB,bodyB,contactSolverCfg);
	}
}

//#define TRY_RANDOMIZE_CONSTRAINT_ORDER

void RigidBodies::applyImpulse(const float timeStep)
{
	ContactSolverConfig contactSolverCfg;
	contactSolverCfg.timeStep = timeStep;
	contactSolverCfg.separateBias = worldProperty.separateBias;
	contactSolverCfg.deformMeshEnable = worldProperty.deformMeshEnable;

#ifdef TRY_RANDOMIZE_CONSTRAINT_ORDER
	uint32_t *idPool = (uint32_t*)mPool->allocate(sizeof(uint32_t)*numContactPairs);
	for(int i=0;i<numContactPairs;i++) {
		idPool[i] = i;
	}
	for(int i=0;i<numContactPairs;i++) {
		int j = rand()%numContactPairs;
		int tmp = idPool[i];
		idPool[i] = idPool[j];
		idPool[j] = tmp;
	}
#endif

	for(uint32_t i=0;i<numContactPairs;i++) {
#ifdef TRY_RANDOMIZE_CONSTRAINT_ORDER
		ContactPair *contactPair = getContactPair(idPool[i]);
#else
		ContactPair *contactPair = getContactPair(i);
#endif
		TrbState &stateA = states[contactPair->stateIndex[0]];
		TrbState &stateB = states[contactPair->stateIndex[1]];
		funcTbl_applyImpulse[stateA.moveType+stateA.sleeping][stateB.moveType+stateB.sleeping](*contactPair,stateA,stateB,contactSolverCfg);
	}

#ifdef TRY_RANDOMIZE_CONSTRAINT_ORDER
	mPool->deallocate(idPool);
#endif
}

void RigidBodies::preJoint(const float timeStep)
{
	JointSolverConfig jointSolverCfg;
	jointSolverCfg.timeStep = timeStep;
	jointSolverCfg.deformMeshEnable = worldProperty.deformMeshEnable;

	for(uint32_t i=0;i<numJoints;i++) {
		if(!joints[i].isActive()) continue;
		TrbState &stateA = states[joints[i].stateIndexA];
		TrbState &stateB = states[joints[i].stateIndexB];
		TrbDynBody &bodyA = bodies[stateA.trbBodyIdx];
		TrbDynBody &bodyB = bodies[stateB.trbBodyIdx];
		funcTbl_preJoint[stateA.moveType+stateA.sleeping][stateB.moveType+stateB.sleeping](joints[i],stateA,bodyA,stateB,bodyB,jointSolverCfg);
	}
}

void RigidBodies::applyJoint(const float timeStep)
{
	JointSolverConfig jointSolverCfg;
	jointSolverCfg.timeStep = timeStep; 
	jointSolverCfg.deformMeshEnable = worldProperty.deformMeshEnable;

#ifdef TRY_RANDOMIZE_CONSTRAINT_ORDER
	uint32_t *idPool = (uint32_t*)mPool->allocate(sizeof(uint32_t)*numJoints);
	for(int i=0;i<numJoints;i++) {
		idPool[i] = i;
	}
	for(int i=0;i<numJoints;i++) {
		int j = rand()%numJoints;
		int tmp = idPool[i];
		idPool[i] = idPool[j];
		idPool[j] = tmp;
	}
#endif

	for(uint32_t i=0;i<numJoints;i++) {
#ifdef TRY_RANDOMIZE_CONSTRAINT_ORDER
		Joint &joint = joints[idPool[i]];
#else
		Joint &joint = joints[i];
#endif
		if(joint.isActive()) {
			TrbState &stateA = states[joint.stateIndexA];
			TrbState &stateB = states[joint.stateIndexB];
			funcTbl_applyJoint[stateA.moveType+stateA.sleeping][stateB.moveType+stateB.sleeping](joint,stateA,stateB,jointSolverCfg);
		}
	}

#ifdef TRY_RANDOMIZE_CONSTRAINT_ORDER
	mPool->deallocate(idPool);
#endif
}

void RigidBodies::postResponse()
{
	for(uint32_t i=0;i<numInstances;i++) {
		TrbState &state = states[i];

		state.setLinearVelocity(state.getLinearVelocity() + state.getDeltaLinearVelocity());
		state.setAngularVelocity(state.getAngularVelocity() + state.getDeltaAngularVelocity());
		state.setDeltaLinearVelocity(Vector3(0.0f));
		state.setDeltaAngularVelocity(Vector3(0.0f));

		if(length(state.getLinearVelocity()) > worldProperty.maxLinearVelocity) {
			state.setLinearVelocity(normalize(state.getLinearVelocity()) * worldProperty.maxLinearVelocity);
		}
		if(length(state.getAngularVelocity()) > worldProperty.maxAngularVelocity) {
			state.setAngularVelocity(normalize(state.getAngularVelocity()) * worldProperty.maxAngularVelocity);
		}
	}
}

void RigidBodies::applySpring()
{
	for(uint32_t i=0;i<numSprings;i++) {
		if(!springs[i].active) continue;

		uint16_t bodyA = springs[i].stateIndexA;
		uint16_t bodyB = springs[i].stateIndexB;

		Vector3 rA = rotate(statesBuffer[readBuffer][bodyA].getOrientation(),springs[i].anchorA);
		Vector3 rB = rotate(statesBuffer[readBuffer][bodyB].getOrientation(),springs[i].anchorB);

		Vector3 pA = statesBuffer[readBuffer][bodyA].getPosition() + rA;
		Vector3 pB = statesBuffer[readBuffer][bodyB].getPosition() + rB;

		Vector3 distance = pB - pA;
		
		float distanceSqr = lengthSqr(distance);
		if(distanceSqr < 1.0e-10f) continue;
		
		Vector3 velocityA = statesBuffer[readBuffer][bodyA].getLinearVelocity() + cross(statesBuffer[readBuffer][bodyA].getAngularVelocity(),rA);
		Vector3 velocityB = statesBuffer[readBuffer][bodyB].getLinearVelocity() + cross(statesBuffer[readBuffer][bodyB].getAngularVelocity(),rB);

		Vector3 relativeVelocity = velocityB - velocityA;
		
		float lenDiff = sqrtf(distanceSqr) - (springs[i].length);

		if(lenDiff < 1.0e-10f) continue;

		Vector3 springVector = normalize(distance);
		Vector3 springForce = -springVector * (springs[i].ks * lenDiff + springs[i].kd * dot(relativeVelocity,springVector));
		if(statesBuffer[readBuffer][bodyA].getMoveTypeBits() & MOVE_TYPE_DYNAMIC ) {
			applyForceByPosition(bodyA,-springForce,pA);
		}
		if(statesBuffer[readBuffer][bodyB].getMoveTypeBits() & MOVE_TYPE_DYNAMIC ) {
			applyForceByPosition(bodyB,springForce,pB);
		}
	}
}

// -------------------------------------------------------
// Sleep & Wakeup

void RigidBodies::sleepOrWakeup()
{
	float slvel = worldProperty.sleepLinearVelocity*worldProperty.sleepLinearVelocity;
	float savel = worldProperty.sleepAngularVelocity*worldProperty.sleepAngularVelocity;
	float wlvel = worldProperty.wakeLinearVelocity*worldProperty.wakeLinearVelocity;
	float wavel = worldProperty.wakeAngularVelocity*worldProperty.wakeAngularVelocity;

	for(uint32_t i = 0; i < numInstances; i++) {
		if(states[i].getMoveTypeBits() & MOVE_TYPE_CAN_SLEEP) {
			float dX = lengthSqr(states[i].getPosition()-prevStates[i].getPosition());
			float dQ = length(states[i].getOrientation()-prevStates[i].getOrientation());
			float lV = lengthSqr(states[i].getLinearVelocity());
			float lO = lengthSqr(states[i].getAngularVelocity());

			if(states[i].isAwake()) {
				// アクティブ→スリープチェック
				if( lV < slvel && 
					lO < savel && 
					dX < slvel && 
					dQ < worldProperty.sleepAngularVelocity) {
					states[i].incrementSleepCount();
				}
				else {
					states[i].resetSleepCount();
				}
				
				if(worldSleepCount % worldProperty.sleepInterval == 0 && states[i].getSleepCount() > worldProperty.sleepCount) {
					states[i].sleep();
				}
			}
			else {
				// スリープ→アクティブチェック
				if(	lV > wlvel ||
					lO > wavel ||
					dX > wlvel ||
					dQ > worldProperty.wakeAngularVelocity) {
					states[i].wakeup();
				}
			}
		}
	}

	worldSleepCount++;
}

ContactPair	*RigidBodies::getContactPairByStates(uint16_t stateIndexA,uint16_t stateIndexB)
{
	// バイナリサーチ
	SortData searchSort(0);
	setStatePair(searchSort,stateIndexA,stateIndexB);
	uint32_t searchKey = Key(searchSort);
	int left = 0;
	int right = numContactPairs;
	int mid;

	while(left <= right) {
		mid = (left+right)>>1;

		if(searchKey == Key(sortedContactPairs[mid])) {
			return &contactPairs[Pair(sortedContactPairs[mid])];
		}
		else if(Key(sortedContactPairs[mid]) < searchKey) {
			left = mid + 1;
		}
		else {
			right = mid - 1;
		}
	}

	return NULL;
}

///////////////////////////////////////////////////////////////////////////////
// 面ローカル衝突点の更新

void RigidBodies::updateFacetLocal(ContactPoint &cp,uint32_t p,LargeTriMesh *curLargeMesh,LargeTriMesh *preLargeMesh,float timeStep)
{
	SubData &subData = cp.subData;
	TriMesh &islandCur = curLargeMesh->islands[subData.getIslandIndex()];
	TriMesh &islandPre = preLargeMesh->islands[subData.getIslandIndex()];
	MeshFacet &facet = islandCur.facets[subData.getFacetIndex()];

	// 衝突点の更新
	float fs = subData.getFacetLocalS();
	float ft = subData.getFacetLocalT();

	Vector3 pntsCur[3] = {
		islandCur.verts[facet.vertIndices[0]],
		islandCur.verts[facet.vertIndices[1]],
		islandCur.verts[facet.vertIndices[2]],
	};

	Vector3 pntsPre[3] = {
		islandPre.verts[facet.vertIndices[0]],
		islandPre.verts[facet.vertIndices[1]],
		islandPre.verts[facet.vertIndices[2]],
	};

	Vector3 cpCur = pntsCur[0] + fs * (pntsCur[1]-pntsCur[0]) + ft * (pntsCur[2]-pntsCur[0]);
	Vector3 cpPre = pntsPre[0] + fs * (pntsPre[1]-pntsPre[0]) + ft * (pntsPre[2]-pntsPre[0]);
	
	if(p==0) {
		cp.setLocalVelocityA((cpCur - cpPre) / timeStep);
		cp.setLocalPointA(cpCur);
	}
	else {
		cp.setLocalVelocityB((cpCur - cpPre) / timeStep);
		cp.setLocalPointB(cpCur);
	}
}

void RigidBodies::updateFacetLocal(Joint &joint,uint32_t p,LargeTriMesh *curLargeMesh,LargeTriMesh *preLargeMesh,float timeStep)
{
	SubData &subData = joint.subData;
	TriMesh &islandCur = curLargeMesh->islands[subData.getIslandIndex()];
	TriMesh &islandPre = preLargeMesh->islands[subData.getIslandIndex()];
	MeshFacet &facet = islandCur.facets[subData.getFacetIndex()];

	// 衝突点の更新
	float fs = subData.getFacetLocalS();
	float ft = subData.getFacetLocalT();

	Vector3 pntsCur[3] = {
		islandCur.verts[facet.vertIndices[0]],
		islandCur.verts[facet.vertIndices[1]],
		islandCur.verts[facet.vertIndices[2]],
	};

	Vector3 pntsPre[3] = {
		islandPre.verts[facet.vertIndices[0]],
		islandPre.verts[facet.vertIndices[1]],
		islandPre.verts[facet.vertIndices[2]],
	};

	Vector3 cpCur = pntsCur[0] + fs * (pntsCur[1]-pntsCur[0]) + ft * (pntsCur[2]-pntsCur[0]);
	Vector3 cpPre = pntsPre[0] + fs * (pntsPre[1]-pntsPre[0]) + ft * (pntsPre[2]-pntsPre[0]);

	if(p==0) {
		joint.localVelocityA = (cpCur - cpPre) / timeStep;
		joint.anchorA = cpCur;
	}
	else {
		joint.localVelocityB = (cpCur - cpPre) / timeStep;
		joint.anchorB = cpCur;
	}
}

void RigidBodies::updateContactPoints(float timeStep)
{
	for(uint32_t i=0;i<numContactPairs;i++) {
		ContactPair &contact = contactPairs[Pair(sortedContactPairs[i])];
		CollObject *collA = getTrbDynBody(contact.stateIndex[0])->getCollObject();
		CollObject *collB = getTrbDynBody(contact.stateIndex[1])->getCollObject();
		
		PFX_ASSERT(collA->getNumPrims() > 0);
		PFX_ASSERT(collB->getNumPrims() > 0);
		
		if(collA->getDefPrim().getType() == LARGEMESH) {
			for(uint32_t c=0;c<contact.numContacts;c++) {
				ContactPoint &cp = contact.contactPoints[c];
				CollPrim &prim = collA->getDefPrim();
				if(prim.getPreLargeMesh() == 0) continue;
				LargeTriMesh *curLargeMesh = prim.getLargeMesh();
				LargeTriMesh *preLargeMesh = prim.getPreLargeMesh();
				updateFacetLocal(cp,0,curLargeMesh,preLargeMesh,timeStep);
			}
		}
		else if(collB->getDefPrim().getType() == LARGEMESH) {
			for(uint32_t c=0;c<contact.numContacts;c++) {
				ContactPoint &cp = contact.contactPoints[c];
				CollPrim &prim = collB->getDefPrim();
				if(prim.getPreLargeMesh() == 0) continue;
				LargeTriMesh *curLargeMesh = prim.getLargeMesh();
				LargeTriMesh *preLargeMesh = prim.getPreLargeMesh();
				updateFacetLocal(cp,1,curLargeMesh,preLargeMesh,timeStep);
			}
		}
	}
}

void RigidBodies::updateJointPoints(float timeStep)
{
	for(uint32_t i=0;i<numJoints;i++) {
		Joint &joint = joints[i];
		CollObject *collA = getTrbDynBody(joint.stateIndexA)->getCollObject();
		CollObject *collB = getTrbDynBody(joint.stateIndexB)->getCollObject();
		
		PFX_ASSERT(collA->getNumPrims() > 0);
		PFX_ASSERT(collB->getNumPrims() > 0);
		
		if(collA->getDefPrim().getType() == LARGEMESH) {
			CollPrim &prim = collA->getDefPrim();
			if(prim.getPreLargeMesh() == 0) continue;
			LargeTriMesh *curLargeMesh = prim.getLargeMesh();
			LargeTriMesh *preLargeMesh = prim.getPreLargeMesh();
			updateFacetLocal(joint,0,curLargeMesh,preLargeMesh,timeStep);
		}
		else if(collB->getDefPrim().getType() == LARGEMESH) {
			CollPrim &prim = collB->getDefPrim();
			if(prim.getPreLargeMesh() == 0) continue;
			LargeTriMesh *curLargeMesh = prim.getLargeMesh();
			LargeTriMesh *preLargeMesh = prim.getPreLargeMesh();
			updateFacetLocal(joint,1,curLargeMesh,preLargeMesh,timeStep);
		}
	}
}

void RigidBodies::setSleepEnable(bool b)
{
	worldProperty.sleepEnable = b;
	for(uint32_t i = 0; i < numInstances; i++) {
		TrbState &state = statesBuffer[readBuffer][i];
		if(state.getMoveTypeBits() & MOVE_TYPE_DYNAMIC && state.isAsleep()) {
			state.wakeup();
		}
	}
}

void RigidBodies::throwContactCallback()
{
	// Throw contact callback
	if(worldProperty.contactCallback) {
		for(uint32_t i=0;i<numContactPairs;i++) {
			if(CallbackFlag(sortedContactPairs[i])) {
				ContactPair &curPair = contactPairs[Pair(sortedContactPairs[i])];
				worldProperty.contactCallback->onContact(curPair);
			}
		}
	}
}

void RigidBodies::throwSleepCallback()
{
	// Throw sleep callback
	if(worldProperty.sleepCallback) {
		for(uint32_t i=0;i<numInstances;i++) {
			if(states[i].getMoveTypeBits() & MOVE_TYPE_DYNAMIC && states[i].getUseSleepCallback()) {
				if(prevStates[i].isAwake() && states[i].isAsleep()) {
					worldProperty.sleepCallback->onSleep(i);
				}
				else if(prevStates[i].isAsleep() && states[i].isAwake()) {
					worldProperty.sleepCallback->onActive(i);
				}
			}
		}
	}
}

void RigidBodies::printState(TrbState &state,const char *name)
{
	PRINTF("----- state of %s -----\n",name);
	PRINTF("moveType %d sleepCount %d contactFilter self 0x%x target 0x%x trbBodyIdx %d\n",
		state.moveType,state.sleepCount,state.contactFilterSelf,state.contactFilterTarget,state.trbBodyIdx);
	PRINTF("fX %f %f %f\n",(float)state.fX[0],(float)state.fX[1],(float)state.fX[2]);
	PRINTF("fV %f %f %f\n",(float)state.fV[0],(float)state.fV[1],(float)state.fV[2]);
	PRINTF("fQ %f %f %f\n",(float)state.fQ[0],(float)state.fQ[1],(float)state.fQ[2]);
	PRINTF("fOmega %f %f %f\n",(float)state.fOmega[0],(float)state.fOmega[1],(float)state.fOmega[2]);
}


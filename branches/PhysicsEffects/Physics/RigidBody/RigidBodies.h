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

#ifndef __RIGIDBODIES_H__
#define __RIGIDBODIES_H__

#include <stdio.h>
#include <string.h>

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/Base/HeapManager.h"
#include "Physics/Base/PfxFileIO.h"
#include "Physics/RigidBody/common/RigidBodyIO.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"
#include "Physics/RigidBody/common/WorldVolume.h"
#include "Physics/RigidBody/common/TrbDynBody.h"
#include "Physics/RigidBody/common/Contact.h"
#include "Physics/RigidBody/common/CollObject.h"
#include "Physics/RigidBody/common/HeightField.h"
#include "Physics/RigidBody/common/Forces.h"
#include "Physics/RigidBody/common/Joint.h"
#include "Physics/RigidBody/common/Spring.h"
#include "Physics/RigidBody/common/ParallelGroup.h"

class RigidBodyTaskMulti;
//#ifndef WIN32
//	#include "Physics/TaskUtil/RigidBodyTaskMulti.h"
//#endif

///////////////////////////////////////////////////////////////////////////////
// 剛体作成時に使用するパラメータ

// 剛体パラメータ
struct RigidBodyProperty {
	float		mass;				// 質量
	Matrix3		inertia;			// 慣性テンソル
	float		friction;			// 摩擦係数
	float		restitution;		// 反発係数
	CollObject	*collObject;		// 剛体形状
	
	RigidBodyProperty()
	{
		mass = 0.0f;
		friction = 0.6f;
		restitution = 0.2f;
		inertia = Matrix3::identity();
		collObject = NULL;
	}
};

// 剛体のインスタンス作成時に使用するパラメータ
struct InstanceProperty {
	uint8_t moveType;				// 動作の種類
	uint8_t sleeping;				// 初期状態でスリープ
	uint32_t contactFilterSelf;		// 自分のコンタクトフィルター
	uint32_t contactFilterTarget;	// 衝突先コンタクトフィルター
	Vector3 position;				// 位置
	Quat	orientation;			// 回転
	Vector3 velocity;				// 速度
	Vector3 angularVelocity;		// 角速度
	TrbDynBody *rigidBody;			// 剛体
	float linearDamping;
	float angularDamping;

	InstanceProperty()
	{
		moveType = MoveTypeFixed;
		sleeping = 0;
		contactFilterSelf = 0xffffffff;
		contactFilterTarget = 0xffffffff;
		position = velocity = angularVelocity = Vector3(0.0f);
		orientation = Quat::identity();
		rigidBody = NULL;
		linearDamping = 1.0f;
		angularDamping = 0.99f;
	}
};

///////////////////////////////////////////////////////////////////////////////
// ジョイント作成時に使用するパラメータ
// ※ パラメータの内容はジョイントの種類によって意味が異なります

struct JointProperty {
	uint8_t		jointType;				// ジョイントの種類
	uint16_t	parentBody; 			// 親-剛体インスタンス
	uint16_t	childBody; 				// 子-剛体インスタンス
	Vector3		anchor;					// 基点
	Vector3		axis;					// 軸

	float		lowerLimit1;			// 可動範囲の下限
	float		upperLimit1;			// 可動範囲の上限
	float		lowerLimit2;			// 可動範囲の下限
	float		upperLimit2;			// 可動範囲の上限
	float		distance;				// Distanceジョイントの距離
	
	// 共通パラメータ
	float		linearDamping;			// 並進ダンピング
	float		angularDamping;			// 回転ダンピング
	float		linearImpulseWeight;	// 並進インパルスウェイト
	float		angularImpulseWeight;	// 回転インパルスウェイト
	float		linearBias;				// 並進バイアス
	float		angularBias;			// 回転バイアス
	float		maxLinearImpulse;		// 最大並進インパルス
	float		maxAngularImpulse;		// 最大回転インパルス
	float		breakableLimit;			// ジョイントを破壊 0.0f=無効
	bool		warmStarting[6];		// Warm Staring

	JointProperty()
	{
		jointType = JointTypeBall;
		parentBody = childBody = 0;
		anchor = Vector3(0.0f);
		axis = Vector3(1.0f,0.0f,0.0f);
		lowerLimit1 = upperLimit1 = 0.0f;
		lowerLimit2 = upperLimit2 = 0.0f;
		linearDamping = angularDamping = 0.0f;
		linearImpulseWeight = angularImpulseWeight = 1.0f;
		linearBias = 0.2f;
		angularBias = 0.2f;
		maxLinearImpulse = 10000.0f;
		maxAngularImpulse = 10000.0f;
		breakableLimit = 0.0f;
		distance = 0.0f;
		warmStarting[0] = false;
		warmStarting[1] = false;
		warmStarting[2] = false;
		warmStarting[3] = false;
		warmStarting[4] = false;
		warmStarting[5] = false;
	}
};

///////////////////////////////////////////////////////////////////////////////
// ワールド作成時に使用するパラメータ

class ContactCallback
{
public:
	ContactCallback() {}
	virtual ~ContactCallback() {}

	virtual void onContact(ContactPair &pair) = 0;
};

class SleepCallback
{
public:
	SleepCallback() {}
	virtual ~SleepCallback() {}

	virtual void onSleep(uint16_t stateIndex) = 0;
	virtual void onActive(uint16_t stateIndex) = 0;
};

struct WorldProperty {
	// オブジェクトの最大数
	uint32_t maxInstances;			// 剛体インスタンスの最大数
	uint32_t maxDynBodies;			// 剛体の最大数
	uint32_t maxPrimitives;			// 形状の最大数
	uint32_t maxJoints;				// ジョイントの最大数
	uint32_t maxSprings;			// スプリングの最大数
	uint32_t maxContactPairs;		// コンタクトペアの最大数

	// ワールド
	Vector3  worldCenter;			// ワールドの中心
	Vector3  worldExtent;			// ワールドの広さ
	Vector3  gravity;				// 重力

	// シミュレーション
	uint8_t subStepCount;			// サブステップ数
	uint8_t contactIteration;		// コンタクト演算の反復回数
	uint8_t jointIteration;			// ジョイント演算の反復回数
	float maxLinearVelocity;		// 速度の最大値
	float maxAngularVelocity;		// 角速度の最大値
	float separateBias;				// 接触している剛体を引き離すためのバイアス値

	// スリープ
	bool  sleepEnable;				// スリープ機能のON/OFF
	uint16_t sleepCount;			// スリープに入るカウント
	uint16_t sleepInterval;			// スリープチェックインターバル
	float sleepLinearVelocity;		// スリープに入るための並進速度
	float sleepAngularVelocity;		// スリープに入るための角速度
	float wakeLinearVelocity;		// 起きるための並進速度
	float wakeAngularVelocity;		// 起きるための角速度
	
	// 機能
	bool ccdEnable;					// 連続的衝突判定のON/OFF
	bool deformMeshEnable;			// 変形メッシュのON/OFF

	// コールバック
	ContactCallback *contactCallback;
	SleepCallback   *sleepCallback;

	WorldProperty()
	{
		maxInstances = 550;
		maxDynBodies = 40;
		maxPrimitives = 200;
		maxJoints = 600;
		maxSprings = 200;
		maxContactPairs = 5000;
		worldExtent = Vector3(200.0f);
		worldCenter = Vector3(0.0f,90.0f,0.0f);
		gravity = Vector3(0.0f,-9.8f,0.0f);
		subStepCount = 1;
		contactIteration = 5;
		jointIteration = 8;
		maxLinearVelocity = 500.0f;
		maxAngularVelocity = 100.0f;
		separateBias = 0.1f;
		sleepEnable = true;
		sleepLinearVelocity = 0.1f;
		sleepAngularVelocity = 0.1f;
		sleepCount = 100;
		sleepInterval = 300;
		wakeLinearVelocity = 0.2f;
		wakeAngularVelocity = 0.2f;
		ccdEnable = false;
		deformMeshEnable = false;
		contactCallback = NULL;
		sleepCallback = NULL;
	}
};

///////////////////////////////////////////////////////////////////////////////
// FindAabbOverlapUtil用コールバッククラス

class AabbOverlapCallback
{
public:
	AabbOverlapCallback() {}
	virtual ~AabbOverlapCallback() {}

	virtual void onOverlap(uint16_t stateIndex) = 0;
};


///////////////////////////////////////////////////////////////////////////////
// RigidBodeisクラス

class RigidBodies
{
friend class HeightFluid;
friend class RayCast;
friend class Particles;
friend class SoftBodies;

private:
	HeapManager *mPool;

#ifndef WIN32
	RigidBodyTaskMulti* mRBTask;
	int mRBTaskID;
#endif
	uint32_t debugFlag;

	uint8_t writeBuffer;
	uint8_t readBuffer;
	TrbState *states;

	// -------------------------------------------------------
	// World Parameter
	
	WorldProperty worldProperty;
	
	uint32_t		numInstances;
	uint32_t		numBodies;
	uint32_t		numCollObjs;
	uint32_t		numPrims;
	uint32_t		numJoints;
	uint32_t		numSprings;
	uint32_t		numContactPairs;
	uint32_t		sizeContactTable;

	Forces     		*forces				__attribute__ ((aligned(16)));
	TrbState   		*statesBuffer[2]	__attribute__ ((aligned(16)));
	TrbState 		*prevStates			__attribute__ ((aligned(16)));
	TrbDynBody		*bodies				__attribute__ ((aligned(16)));
	CollObject		*collObjs			__attribute__ ((aligned(16)));
	CollPrim		*prims				__attribute__ ((aligned(16)));
	Joint			*joints				__attribute__ ((aligned(16)));
	Spring			*springs			__attribute__ ((aligned(16)));
	ContactPair		*contactPairs		__attribute__ ((aligned(16)));
	SortData		*sortedContactPairs	__attribute__ ((aligned(16)));
	uint32_t		*nonContactPair		__attribute__ ((aligned(16)));
	uint32_t		*userData			__attribute__ ((aligned(16)));

	// 再利用プール
	uint16_t		numBodiesPool;
	uint16_t		numCollsPool;
	uint16_t		numPrimsPool;
	uint16_t		numStatesPool;
	uint16_t		numJointsPool;
	uint16_t		numSpringsPool;

	uint16_t		*bodiesPool			__attribute__ ((aligned(16)));
	uint16_t		*collsPool			__attribute__ ((aligned(16)));
	uint16_t		*primsPool			__attribute__ ((aligned(16)));
	uint16_t		*statesPool			__attribute__ ((aligned(16)));
	uint16_t		*jointsPool			__attribute__ ((aligned(16)));
	uint16_t		*springsPool		__attribute__ ((aligned(16)));

	// -------------------------------------------------------
	// ContactPair Table
	
	inline void clearNonContactPair();
	inline bool isCollidablePair(uint16_t i,uint16_t j);
	inline bool isCollidable(uint16_t stateIndexA,uint32_t selfA,uint32_t targetA,uint16_t stateIndexB,uint32_t selfB,uint32_t targetB);

	// -------------------------------------------------------
	// Broad Phase

	WorldVolume worldVolume;

	void mergePairs(SortData *newPairs,uint32_t &numNewPairs);
	void broadPhase(float timeStep);

#ifndef WIN32
	void assignStatesSPU(float timeStep,SortData *movAabbArray,uint32_t &numMovAabb,SortData *fixAabbArray,uint32_t &numFixAabb,int &chkAxis);
	void detectPairsSPU(SortData *newPairs,uint32_t &numNewPairs,uint32_t maxNewPairs,SortData *movAabbArray,uint32_t numMovAabb,SortData *fixAabbArray,uint32_t numFixAabb,int chkAxis);
	void mergePairsSPU(SortData *newPairs,uint32_t &numNewPairs);
	void addNewPairsSPU(SortData *newPairs,uint32_t numNewPairs);
	void broadPhaseSPU(float timeStep);
#endif

	// -------------------------------------------------------
	// Integrate

	void integrate(float timeStep,bool lastSubStep);

#ifndef WIN32
	void integrateSPU(float timeStep,bool lastSubStep);
#endif

	// -------------------------------------------------------
	// Refresh Contact Pairs

	uint32_t refreshContactPairs();

#ifndef WIN32
	uint32_t refreshContactPairsSPU();
#endif

	// -------------------------------------------------------
	// Collision & Response

	void preResponse(const float timeStep);
	void postResponse();
	void applyImpulse(const float timeStep);

	void preJoint(const float timeStep);
	void applyJoint(const float timeStep);
	
	void applySpring();

	void detectCollisions(float timeStep);
	void solveConstraints(float timeStep);

#ifndef WIN32
	void splitConstraints(SolverInfo *info,SolverGroup *groups,SortData *pairs,uint32_t numPairs);
	void splitConstraintsSPU(SolverInfo *info,SolverGroup *groups,SortData *pairs,uint32_t numPairs);
	uint32_t createJointPairsSPU(SortData *pairs);
	void detectCollisionsSPU(float timeStep);
	void solveConstraintsSPU(float timeStep);
	void printSolverInfo(SolverInfo *info,SolverGroup *group,SortData *pairs);
#endif

	// -------------------------------------------------------
	// Sleep & Wakeup
	
	// しばらく動かないオブジェクトをスリープ状態へ移行
	// また速度変化が閾値を超えた剛体をスリープ状態から起こす

	uint32_t worldSleepCount;

	void sleepOrWakeup();

#ifndef WIN32
	void sleepOrWakeupSPU();
#endif

	// -------------------------------------------------------
	// Deform Mesh

	void updateContactPoints(float timeStep);
	void updateJointPoints(float timeStep);
	void updateFacetLocal(ContactPoint &cp,uint32_t p,LargeTriMesh *curLargeMesh,LargeTriMesh *preLargeMesh,float timeStep);
	void updateFacetLocal(Joint &joint,uint32_t p,LargeTriMesh *curLargeMesh,LargeTriMesh *preLargeMesh,float timeStep);

	// -------------------------------------------------------
	// Callback

	void throwSleepCallback();
	void throwContactCallback();

	// -------------------------------------------------------
	// Joint

	int createJoint();

	// -------------------------------------------------------
	// Initialize
	
	void allocateBuffers();
	void deallocateBuffers();
	
	void setupWorldSize();
	
	RigidBodies() {}
public:

#ifndef WIN32
	RigidBodies(RigidBodyTaskMulti *RBTask,int taskID,HeapManager *pool);
#else
	RigidBodies(HeapManager *pool);
#endif

	virtual ~RigidBodies()
	{
		deallocateBuffers();
	}

	void reset();
	void setup();
	
	void printWorld();
	void printBufSize();
	uint32_t calcBufSize();

	// -------------------------------------------------------
	// Simulation Setup

	void setupSimulate();
	void finalizeSimulate();

	// -------------------------------------------------------
	// PPU Simulation

	void ppuSimulate(float timeStep,uint32_t flag=0);

	// -------------------------------------------------------
	// SPU Simulation

#ifndef WIN32
	
	// IO Parameter
	IOParamAssignStates	mAssignStatesIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	IOParamDetectPairs	mDetectPairsIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	IOParamMergePairs	mMergePairsIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	IOParamRefreshPairs	mRefreshPairsIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	IOParamSort			mSortIO __attribute__ ((aligned(16)));
	IOParamIntegrate	mIntegrateIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	IOParamCollision	mCollisionIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	IOParamSolver		mSolverIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	IOParamPostResponse	mPostResponseIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	IOParamAddNewPairs  mAddNewPairsIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	IOParamSplitConstraints mSplitConstraintsIO __attribute__ ((aligned(16)));
	IOParamCreateJointPairs mCreateJointPairsIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));

	//int getNumTasks() {return ((RigidBodyTaskMulti*)mRBTask)->getNumTasks();}

	void spuSimulate(float timeStep,uint32_t flag=0);
#endif

	// -------------------------------------------------------
	// AABB Overlap Utility
	
private:
	int mCheckAxis;
	int mNumAabbArray;
	SortData *mAabbArray;

public:
	void setupAabbOverlapUtil();
	void finalizeAabbOverlapUtil();

	void ppuFindAabbOverlap(Vector3 aabbMin,Vector3 aabbMax,uint32_t self,uint32_t target,AabbOverlapCallback *findAabbOverlap);

#ifndef WIN32
	IOParamFindAabbOverlap	mFindAabbOverlapIO[PFX_MAX_SPUS] __attribute__ ((aligned(16)));
	void spuFindAabbOverlap(Vector3 aabbMin,Vector3 aabbMax,uint32_t self,uint32_t target,AabbOverlapCallback *findAabbOverlap);
#endif

public:

	// -------------------------------------------------------
	// Create / Delete

	// ＜削除の注意点＞
	// ・削除されたオブジェクトはプールされ次回の作成時に再利用されます
	// ・削除されたオブジェクトは非アクティブ化され、オブジェクトの総数は変化しません
	// ・剛体やジョイントの整合性はアプリケーション側で対応する必要があります
	//   （ジョイント接続された剛体が削除された場合、ジョイントも削除または非アクティブにしてください）
	
	// Todo: private:
	int createCollPrim();
	bool deleteCollPrim(int primId);

	// 剛体の形状の作成と削除
	// 作成に失敗したときはNULLを返す
	CollObject*	createCollObject();
	CollObject*	createCollObject(int numPrims); // Todo:
	bool deleteCollObject(CollObject *coll);
	
	// 剛体の作成と削除
	// 作成に失敗したときはNULLを返す
	TrbDynBody*	createRigidBody(const RigidBodyProperty &param);
	bool deleteRigidBody(TrbDynBody *rigidbody);

	// 剛体のインスタンス作成と削除
	// 成功したときはインスタンスのインデックス、作成に失敗したときは-1を返す
	int	createInstance(const InstanceProperty &param);
	bool deleteInstance(int stateIndex);

	// 剛体インスタンスのクローンを作成
	// 成功したときはインスタンスのインデックス、作成に失敗したときは-1を返す
	int	cloneInstance(uint32_t instance);

	// 剛体インスタンスが存在するかをチェック
	bool instanceExists(uint32_t stateIndex) {return !statesBuffer[readBuffer][stateIndex].isDeleted();}
	
	// ジョイントの作成と削除
	// 成功したときはジョイントのインデックス、作成に失敗したときは-1を返す
	int createJoint(const JointProperty &jointParam);
	bool deleteJoint(int jointIndex);

	// スプリングの作成と削除 ks = バネ係数 kd = ダンピング
	// 成功したときはスプリングのインデックス、作成に失敗したときは-1を返す
	int createSpring(float length,float ks,float kd,uint16_t stateIndexA,uint16_t stateIndexB,const Vector3 &worldAnchor);
	int createSpring(float length,float ks,float kd,uint16_t stateIndexA,uint16_t stateIndexB,const Vector3 &localAnchorA,const Vector3 &localAnchorB);
	bool deleteSpring(int springIndex);

	// -------------------------------------------------------
	// World

	// ※ ワールド設定を変更した後は必ずreset()を呼び出してください
	void		getWorldProperty(WorldProperty &worldProp) {worldProp = worldProperty;}
	void		setWorldProperty(WorldProperty &worldProp) {worldProperty = worldProp;}
	
	Vector3		getGravity() {return worldProperty.gravity;}
	void		setGravity(const Vector3 &gravity) {worldProperty.gravity = gravity;}

	uint32_t	getSubStepCount() {return worldProperty.subStepCount;}
	void		setSubStepCount(uint32_t i) {worldProperty.subStepCount = (uint8_t)i;}

	uint8_t		getContactIteration() {return worldProperty.contactIteration;}
	void		setContactIteration(uint8_t i) {worldProperty.contactIteration = i;}

	uint8_t		getJointIteration() {return worldProperty.jointIteration;}
	void		setJointIteration(uint8_t i) {worldProperty.jointIteration = i;}

	float		getMaxLinearVelocity() {return worldProperty.maxLinearVelocity;}
	void		setMaxLinearVelocity(float value) {worldProperty.maxLinearVelocity = value;}

	float		getMaxAngularVelocity() {return worldProperty.maxAngularVelocity;}
	void		setMaxAngularVelocity(float value) {worldProperty.maxAngularVelocity = value;}

	float		getSeparateBias() {return worldProperty.separateBias;}
	void		setSeparateBias(float value) {worldProperty.separateBias = value;}

	bool		getSleepEnable() {return worldProperty.sleepEnable;}
	void		setSleepEnable(bool b);

	bool		getCCDEnable() {return worldProperty.ccdEnable;}
	void		setCCDEnable(bool b)  {worldProperty.ccdEnable = b;}

	bool		getDeformMeshEnable() {return worldProperty.deformMeshEnable;}
	void		setDeformMeshEnable(bool b)  {worldProperty.deformMeshEnable = b;}

	void		getWorldSize(Vector3 &center,Vector3 &extent);
	void		setWorldSize(const Vector3 &center,const Vector3 &extent);

	// -------------------------------------------------------
	// Rigid Body
	
	// 非衝突ペアを登録・解除
	void		appendNonContactPair(uint16_t stateIndexA,uint16_t stateIndexB);
	void		removeNonContactPair(uint16_t stateIndexA,uint16_t stateIndexB);
	bool		checkNonContactPair(uint16_t stateIndexA,uint16_t stateIndexB);
	
	inline uint32_t	getNonContactPair(int index) {return nonContactPair[index];}
	inline void		setNonContactPair(int index,uint32_t v) {nonContactPair[index] = v;}
	
	inline uint32_t	getTrbDynBodyCount() {return numBodies;}
	inline uint32_t	getInstanceCount() {return numInstances;}
	inline uint32_t	getJointCount() {return numJoints;}
	inline uint32_t	getSpringCount() {return numSprings;}
	inline uint32_t	getContactCount() {return numContactPairs;}

	inline TrbDynBody	*getTrbDynBody(int stateIndex) {return &bodies[statesBuffer[readBuffer][stateIndex].trbBodyIdx];}
	inline CollObject	*getCollObject(int stateIndex) {return getTrbDynBody(stateIndex)->getCollObject();}
	inline TrbDynBody	*getTrbDynBodyByIndex(int index) {return &bodies[index];}
	inline Forces		*getForce(int stateIndex) {return &forces[stateIndex];}
	inline Joint		*getJoint(int jointIndex) {return &joints[jointIndex];}
	inline Spring		*getSpring(int springIndex) {return &springs[springIndex];}
	inline ContactPair	*getContactPair(int contactIndex) {return &contactPairs[Pair(sortedContactPairs[contactIndex])];}
	ContactPair			*getContactPairByStates(uint16_t stateIndexA,uint16_t stateIndexB);

	inline TrbState	*getState(int stateIndex) {return &statesBuffer[readBuffer][stateIndex];}
	inline void		setState(int stateIndex,const TrbState &state) {statesBuffer[readBuffer][stateIndex] = state;}

	inline TrbState	*getOldState(int stateIndex) {return &prevStates[stateIndex];}

	bool			isAsleep(int stateIndex) {return statesBuffer[readBuffer][stateIndex].isAsleep();}
	bool			isAwake(int stateIndex) {return statesBuffer[readBuffer][stateIndex].isAwake();}

	void			wakeup(int stateIndex) {statesBuffer[readBuffer][stateIndex].wakeup();}
	void			sleep(int stateIndex) {statesBuffer[readBuffer][stateIndex].sleep();}

	float			getLinearDamping(int stateIndex) {return statesBuffer[readBuffer][stateIndex].linearDamping;}
	void			setLinearDamping(int stateIndex,float value) {statesBuffer[readBuffer][stateIndex].linearDamping = value;}

	float			getAngularDamping(int stateIndex) {return statesBuffer[readBuffer][stateIndex].angularDamping;}
	void			setAngularDamping(int stateIndex,float value) {statesBuffer[readBuffer][stateIndex].angularDamping = value;}

	inline uint8_t	getMoveType(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getMoveType();}
	inline void		setMoveType(int stateIndex,uint8_t moveType) {statesBuffer[readBuffer][stateIndex].setMoveType(moveType);}

	inline uint32_t	getContactFilterSelf(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getContactFilterSelf();}
	inline void		setContactFilterSelf(int stateIndex,uint32_t filter) {statesBuffer[readBuffer][stateIndex].setContactFilterSelf(filter);}

	inline uint32_t	getContactFilterTarget(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getContactFilterTarget();}
	inline void		setContactFilterTarget(int stateIndex,uint32_t filter) {statesBuffer[readBuffer][stateIndex].setContactFilterTarget(filter);}

	inline void		setContactFilter(int stateIndex,uint32_t filter) {setContactFilterSelf(stateIndex,filter);setContactFilterTarget(stateIndex,filter);}

	inline bool		getUseContactCallback(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getUseContactCallback()!=0;}
	inline void		setUseContactCallback(int stateIndex,bool b) {statesBuffer[readBuffer][stateIndex].setUseContactCallback(b?1:0);}

	inline bool		getUseSleepCallback(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getUseSleepCallback()!=0;}
	inline void		setUseSleepCallback(int stateIndex,bool b) {statesBuffer[readBuffer][stateIndex].setUseSleepCallback((uint8_t)b);}

	inline bool		getUseCcd(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getUseCcd()!=0;}
	inline void		setUseCcd(int stateIndex,bool b) {statesBuffer[readBuffer][stateIndex].setUseCcd((uint8_t)b);}

	inline bool		getUseSleep(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getUseSleep()!=0;}
	inline void		setUseSleep(int stateIndex,bool b) {statesBuffer[readBuffer][stateIndex].setUseSleep((uint8_t)b);}

	inline uint32_t	getUserData(int stateIndex) {return userData[stateIndex];}
	inline void		setUserData(int stateIndex,uint32_t data) {userData[stateIndex] = data;}

	inline Vector3	getWorldPosition(int stateIndex,const Vector3 &localPos);
	inline Vector3	getLocalPosition(int stateIndex,const Vector3 &worldPos);
	inline Vector3	getPosition(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getPosition();}
	inline void		setPosition(int stateIndex,const Vector3 &position);

	inline Quat		getOrientation(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getOrientation();}
	inline void		setOrientation(int stateIndex,const Quat &orientation);

	inline Vector3	getVelocity(int stateIndex)  {return statesBuffer[readBuffer][stateIndex].getLinearVelocity();}
	inline void		setVelocity(int stateIndex,const Vector3 &velocity);

	inline Vector3	getAngularVelocity(int stateIndex) {return statesBuffer[readBuffer][stateIndex].getAngularVelocity();}
	inline void		setAngularVelocity(int stateIndex,const Vector3 &angularVelocity);

	inline void		applyForce(int stateIndex,const Forces &force);
	inline void		applyForceByPosition(int stateIndex,const Vector3 &force,const Vector3 &worldPosition);

	inline void		applyLinearImpulse(int stateIndex,const Vector3 &impulse);
	inline void		applyAngularImpulse(int stateIndex,const Vector3 &impulse);
	inline void		applyImpulseByPosition(int stateIndex,const Vector3 &impulse,const Vector3 &worldPosition);

	inline void		updateAABB(int stateIndex);
	inline void		updateAABBCcd(int stateIndex,float timeStep);

	// キーフレーム剛体の位置を変更する（過去位置との差分により速度を算出します）
	inline void		movePosition(int stateIndex,const Vector3 &position,float timeStep);
	inline void		moveOrientation(int stateIndex,const Quat &orientation,float timeStep);

	// アニメーションジョイントのターゲットフレームの指定
	inline void		setPoseLocal(int jointIndex,const Quat &poseLocal); // ポーズを親フレーム座標系で指定
	inline void		setPoseWorld(int jointIndex,const Quat &poseWorld);	// ポーズをワールド座標系で指定
	
	// ジョイントパラメータ
	inline float	getJointLinearImpulseWeight(int jointIndex) {return joints[jointIndex].linearImpulseWeight;}
	inline void		setJointLinearImpulseWeight(int jointIndex,float weight) {joints[jointIndex].linearImpulseWeight = weight;}

	inline float	getJointAngularImpulseWeight(int jointIndex) {return joints[jointIndex].angularImpulseWeight;}
	inline void		setJointAngularImpulseWeight(int jointIndex,float weight) {joints[jointIndex].angularImpulseWeight = weight;}

	inline float	getJointLinearDamping(int jointIndex) {return joints[jointIndex].linearDamping;}
	inline void		setJointLinearDamping(int jointIndex,float damping) {joints[jointIndex].linearDamping = damping;}

	inline float	getJointAngularDamping(int jointIndex) {return joints[jointIndex].angularDamping;}
	inline void		setJointAngularDamping(int jointIndex,float damping) {joints[jointIndex].angularDamping = damping;}

	inline float	getJointLinearBias(int jointIndex) {return joints[jointIndex].linearBias;}
	inline void		setJointLinearBias(int jointIndex,float value) {joints[jointIndex].linearBias = value;}

	inline float	getJointAngularBias(int jointIndex) {return joints[jointIndex].angularBias;}
	inline void		setJointAngularBias(int jointIndex,float value) {joints[jointIndex].angularBias = value;}

	// 剛体の初期化
	void setupRigidBody(uint16_t stateIndex);

	// コールバック時の更新に使用
	inline Vector3	getWorldPositionInternal(int stateIndex,const Vector3 &localPos);
	inline Vector3	getLocalPositionInternal(int stateIndex,const Vector3 &worldPos);
	inline Vector3	getPositionInternal(int stateIndex) {return statesBuffer[writeBuffer][stateIndex].getPosition();}
	inline void		setPositionInternal(int stateIndex,const Vector3 &position);

	inline Quat		getOrientationInternal(int stateIndex) {return statesBuffer[writeBuffer][stateIndex].getOrientation();}
	inline void		setOrientationInternal(int stateIndex,const Quat &orientation);

	inline Vector3	getVelocityInternal(int stateIndex)  {return statesBuffer[writeBuffer][stateIndex].getLinearVelocity();}
	inline void		setVelocityInternal(int stateIndex,const Vector3 &velocity);

	inline Vector3	getAngularVelocityInternal(int stateIndex) {return statesBuffer[writeBuffer][stateIndex].getAngularVelocity();}
	inline void		setAngularVelocityInternal(int stateIndex,const Vector3 &angularVelocity);

	inline void		applyLinearImpulseInternal(int stateIndex,const Vector3 &impulse);
	inline void		applyAngularImpulseInternal(int stateIndex,const Vector3 &impulse);
	inline void		applyImpulseByPositionInternal(int stateIndex,const Vector3 &impulse,const Vector3 &worldPosition);

	inline void		updateAABBInternal(int stateIndex);
	inline void		updateAABBCcdInternal(int stateIndex,float timeStep);

	// -------------------------------------------------------
	// Debug

	void printState(TrbState &state,const char *name);
	void printContactPairs();

	// -------------------------------------------------------
	// Snapshot

private:
	PfxFileWriter mWriter;

	void writePreHeader();
	void writePostHeader();
	void writeWorld();
	void writeBodies();
	void writeStates();
	void writeJoints();
	void writeMeshes();
	void writeNonContactPair();
	void writeTrbDynBody(const TrbDynBody &body,uint32_t id);
	void writeTrbState(const TrbState &state,uint32_t id);
	void writeJoint(const Joint &joint);
	void writeConvexMesh(const ConvexMesh &mesh);
	void writeLargeMesh(const LargeTriMesh &largeMesh);

public:
	bool saveSnapshot(const char *filename);
};

inline
void RigidBodies::clearNonContactPair()
{
	memset(nonContactPair,0,sizeof(uint32_t)*sizeContactTable);
}

inline
bool RigidBodies::isCollidablePair(uint16_t i,uint16_t j)
{
	PFX_ASSERT(i != j);
	PFX_ASSERT(i < worldProperty.maxInstances);
	PFX_ASSERT(j < worldProperty.maxInstances);

	uint32_t minIdx = i>j?j:i;
	uint32_t maxIdx = i>j?i:j;
	uint32_t idx = maxIdx * (maxIdx - 1) / 2 + minIdx;
	uint32_t mask = 1L << (idx & 31);
	return (nonContactPair[idx>>5] & mask) == 0;
}

inline
void RigidBodies::updateAABB(int stateIndex)
{
	CollObject &coll = collObjs[statesBuffer[readBuffer][stateIndex].trbBodyIdx];
	statesBuffer[readBuffer][stateIndex].setAuxils(coll.getCenter(),coll.getHalf());
}

inline
void RigidBodies::updateAABBCcd(int stateIndex,float timeStep)
{
	CollObject &coll = collObjs[statesBuffer[readBuffer][stateIndex].trbBodyIdx];
	statesBuffer[readBuffer][stateIndex].setAuxilsCcd(coll.getCenter(),coll.getHalf(),timeStep);
}

inline
bool RigidBodies::isCollidable(uint16_t stateIndexA,uint32_t selfA,uint32_t targetA,uint16_t stateIndexB,uint32_t selfB,uint32_t targetB)
{
	return isCollidablePair(stateIndexA,stateIndexB) && ((selfA&targetB) && (targetA&selfB));
}

inline
void RigidBodies::setPosition(int stateIndex,const Vector3 &position)
{
	statesBuffer[readBuffer][stateIndex].setPosition(position);
	updateAABB(stateIndex);
}

inline
void RigidBodies::setOrientation(int stateIndex,const Quat &orientation)
{
	statesBuffer[readBuffer][stateIndex].setOrientation(orientation);
	updateAABB(stateIndex);
}

inline
void RigidBodies::movePosition(int stateIndex,const Vector3 &position,float timeStep)
{
	TrbState &state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveType()!=MoveTypeKeyframe) return;
	if(state.isAsleep() ) state.wakeup();

	Vector3 prePos = getPosition(stateIndex);
	Vector3 vel = (position - prePos ) / timeStep;

	setVelocity(stateIndex,vel);
	
	if(worldProperty.ccdEnable && getUseCcd(stateIndex)) {
		updateAABBCcd(stateIndex,timeStep);
	}
	else {
		updateAABB(stateIndex);
	}
}

inline
void RigidBodies::moveOrientation(int stateIndex,const Quat &orientation,float timeStep)
{
	TrbState &state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveType()!=MoveTypeKeyframe) return;
	if(state.isAsleep() ) state.wakeup();

	Quat preOri = getOrientation(stateIndex);
	Quat oldOri = orientation;

	if(dot(orientation,preOri) < 0.0f) {
		oldOri = -orientation;
	}

	Quat dq = ( oldOri - preOri ) / timeStep;
	dq = dq * 2.0f * conj(preOri);
	Vector3 omega = dq.getXYZ();

	setAngularVelocity(stateIndex,omega);
	
	if(worldProperty.ccdEnable && getUseCcd(stateIndex)) {
		updateAABBCcd(stateIndex,timeStep);
	}
	else {
		updateAABB(stateIndex);
	}
}

inline
void RigidBodies::setVelocity(int stateIndex,const Vector3 &velocity)
{
	statesBuffer[readBuffer][stateIndex].setLinearVelocity(velocity);
}

inline
void RigidBodies::setAngularVelocity(int stateIndex,const Vector3 &angularVelocity)
{
	statesBuffer[readBuffer][stateIndex].setAngularVelocity(angularVelocity);
}

inline
void RigidBodies::applyForce(int stateIndex,const Forces &force)
{
	TrbState &state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits() & MOVE_TYPE_CAN_SLEEP && state.isAsleep() ) state.wakeup();

	forces[stateIndex].force += force.force;
	forces[stateIndex].torque += force.torque;
}

inline
void RigidBodies::applyForceByPosition(int stateIndex,const Vector3 &force,const Vector3 &worldPosition)
{
	TrbState &state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits() & MOVE_TYPE_CAN_SLEEP && state.isAsleep() ) state.wakeup();

	forces[stateIndex].force += force;
	forces[stateIndex].torque += cross(worldPosition-state.getPosition(),force);
}

inline
void RigidBodies::applyLinearImpulse(int stateIndex,const Vector3 &impulse)
{
	TrbState &state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits() & MOVE_TYPE_CAN_SLEEP && state.isAsleep() ) state.wakeup();

	state.setLinearVelocity(state.getLinearVelocity() + bodies[state.trbBodyIdx].getMassInv() * impulse);
}

inline
void RigidBodies::applyAngularImpulse(int stateIndex,const Vector3 &impulse)
{
	TrbState &state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits() & MOVE_TYPE_CAN_SLEEP && state.isAsleep() ) state.wakeup();

	state.setAngularVelocity(state.getAngularVelocity() + bodies[state.trbBodyIdx].getBodyInertiaInv() * impulse);
}

inline
void RigidBodies::applyImpulseByPosition(int stateIndex,const Vector3 &impulse,const Vector3 &worldPosition)
{
	TrbState &state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits() & MOVE_TYPE_CAN_SLEEP && state.isAsleep() ) state.wakeup();

	state.setLinearVelocity(state.getLinearVelocity() + bodies[state.trbBodyIdx].getMassInv() * impulse);
	state.setAngularVelocity(state.getAngularVelocity() + bodies[state.trbBodyIdx].getBodyInertiaInv() * cross(worldPosition-state.getPosition(),impulse));
}

inline
Vector3 RigidBodies::getWorldPosition(int stateIndex,const Vector3 &localPos)
{
	return statesBuffer[readBuffer][stateIndex].getPosition() + rotate(statesBuffer[readBuffer][stateIndex].getOrientation(),localPos);
}

inline
Vector3 RigidBodies::getLocalPosition(int stateIndex,const Vector3 &worldPos)
{
	return rotate(conj(statesBuffer[readBuffer][stateIndex].getOrientation()),(worldPos - statesBuffer[readBuffer][stateIndex].getPosition()));
}

inline
void RigidBodies::setPoseLocal(int jointIndex,const Quat &poseLocal)
{
	joints[jointIndex].targetFrame = Matrix3(poseLocal);
}

inline
void RigidBodies::setPoseWorld(int jointIndex,const Quat &poseWorld)
{
	Matrix3 ma(getOrientation(joints[jointIndex].stateIndexA));
	Matrix3 mf(joints[jointIndex].frameA);
	joints[jointIndex].targetFrame = transpose(ma * mf) * Matrix3(poseWorld);
}

inline
void RigidBodies::updateAABBInternal(int stateIndex)
{
	CollObject &coll = collObjs[statesBuffer[writeBuffer][stateIndex].trbBodyIdx];
	statesBuffer[writeBuffer][stateIndex].setAuxils(coll.getCenter(),coll.getHalf());
}

inline
void RigidBodies::updateAABBCcdInternal(int stateIndex,float timeStep)
{
	CollObject &coll = collObjs[statesBuffer[writeBuffer][stateIndex].trbBodyIdx];
	statesBuffer[writeBuffer][stateIndex].setAuxilsCcd(coll.getCenter(),coll.getHalf(),timeStep);
}

inline
Vector3 RigidBodies::getWorldPositionInternal(int stateIndex,const Vector3 &localPos)
{
	return statesBuffer[writeBuffer][stateIndex].getPosition() + rotate(statesBuffer[writeBuffer][stateIndex].getOrientation(),localPos);
}

inline
Vector3 RigidBodies::getLocalPositionInternal(int stateIndex,const Vector3 &worldPos)
{
	return rotate(conj(statesBuffer[writeBuffer][stateIndex].getOrientation()),(worldPos - statesBuffer[writeBuffer][stateIndex].getPosition()));
}

inline
void RigidBodies::setPositionInternal(int stateIndex,const Vector3 &position)
{
	statesBuffer[writeBuffer][stateIndex].setPosition(position);
	updateAABB(stateIndex);
}

inline
void RigidBodies::setOrientationInternal(int stateIndex,const Quat &orientation)
{
	statesBuffer[writeBuffer][stateIndex].setOrientation(orientation);
	updateAABB(stateIndex);
}

inline
void RigidBodies::setVelocityInternal(int stateIndex,const Vector3 &velocity)
{
	statesBuffer[writeBuffer][stateIndex].setLinearVelocity(velocity);
}

inline
void RigidBodies::setAngularVelocityInternal(int stateIndex,const Vector3 &angularVelocity)
{
	statesBuffer[writeBuffer][stateIndex].setAngularVelocity(angularVelocity);
}

inline
void RigidBodies::applyLinearImpulseInternal(int stateIndex,const Vector3 &impulse)
{
	TrbState &state = statesBuffer[writeBuffer][stateIndex];
	if(state.getMoveTypeBits() & MOVE_TYPE_CAN_SLEEP && state.isAsleep() ) state.wakeup();

	state.setLinearVelocity(state.getLinearVelocity() + bodies[state.trbBodyIdx].getMassInv() * impulse);
}

inline
void RigidBodies::applyAngularImpulseInternal(int stateIndex,const Vector3 &impulse)
{
	TrbState &state = statesBuffer[writeBuffer][stateIndex];
	if(state.getMoveTypeBits() & MOVE_TYPE_CAN_SLEEP && state.isAsleep() ) state.wakeup();
	
	state.setAngularVelocity(state.getAngularVelocity() + bodies[state.trbBodyIdx].getBodyInertiaInv() * impulse);
}

inline
void RigidBodies::applyImpulseByPositionInternal(int stateIndex,const Vector3 &impulse,const Vector3 &worldPosition)
{
	TrbState &state = statesBuffer[writeBuffer][stateIndex];
	if(state.getMoveTypeBits() & MOVE_TYPE_CAN_SLEEP && state.isAsleep() ) state.wakeup();

	state.setLinearVelocity(state.getLinearVelocity() + bodies[state.trbBodyIdx].getMassInv() * impulse);
	state.setAngularVelocity(state.getAngularVelocity() + bodies[state.trbBodyIdx].getBodyInertiaInv() * cross(worldPosition-state.getPosition(),impulse));
}

#endif /* __RIGIDBODIES_H__ */

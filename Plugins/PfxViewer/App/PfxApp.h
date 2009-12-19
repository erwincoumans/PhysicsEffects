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

#ifndef __PFX_APP_H__
#define __PFX_APP_H__

#include "CommonApp.h"
#include "Physics/RigidBody/RigidBodies.h"
#include "Physics/RayCast/RayCast.h"
#include "../Render/RenderUtil.h"
#include "../../PfxCommonFormat/PfxCommonImport.h"
#include "PfxSnapshotReader.h"

#include <map>

#define MAX_CHARGE	2.0f
#define THROW_POWER	50.0f

class PfxApp : public CommonApp
{
private:
	RenderUtil mRenderUtil;
	RigidBodies *mRigidBodies;
	RayCast *mRayCast;

	// ピック用
	Joint *mPickJoint;
	int	mGroundBody;
	float mPickZ;
	
	// 投擲
	float mChargeTime;
	int mBallBody;
	
	// パフォーマンス計測
	float mFreq;
	float mSimulationTime;
	float mTotalTime;

public:
	PfxApp();
	virtual ~PfxApp();

	// --------------------------------------------------------
	// フレームワーク

	void onRender();
	bool onUpdate();

	// --------------------------------------------------------
	// メモリ
	
	int mMemSize;
	unsigned char *mRigMemPool;
	unsigned char *mRayMemPool;
	HeapManager *mRigHeap;
	HeapManager *mRayHeap;

	void allocatePhysicsMemory();
	void deallocatePhysicsMemory();

	int checkRequiredMemory(WorldProperty &worldProperty);

	// --------------------------------------------------------
	// 物理

	bool onInitPhysicsEngine();
	void onShutdownPhysicsEngine();
	void onInitPhysicsScene();
	void onReleasePhysicsScene();
	void onWaitSimulation();
	void onUpdateSimulation();
	void onStartSimulation();

	// --------------------------------------------------------
	// 描画

	enum DrawFlag {
		DrawFlagLocalAxis   = 0x0001,
		DrawFlagAABB        = 0x0002,
		DrawFlagContact     = 0x0004,
		DrawFlagJoint       = 0x0008,
		DrawFlagShadow      = 0x0010,
		DrawFlagSilhouette  = 0x0020,
		DrawFlagPerf        = 0x0040,
		DrawFlagIslandsAABB = 0x0080,
		DrawFlagIslandsWire = 0x0100,
		DrawFlagInertia     = 0x0200,
	};

	uint32_t mDrawFlag;
	Vector3 mColorArray[256];

	void setDrawFlag(uint32_t flag) {mDrawFlag = flag;}

	void drawObjectsShilhouette();
	void drawObjects();
	void drawDebugInfo();

	// --------------------------------------------------------
	// ピック

	enum ClickMode {
		ClickModePick,
		ClickModeView,
	};

	struct PickInfo {
		Ray		ray;
		string	name;
		string  moveType;
		string  primType;
		float	mass;
		float	friction;
		float	resitution;
		Vector3	aabbHalf;
		Vector3 position;
		Quat	orentation;
		Vector3	linearVelocity;
		Vector3	angularVelocity;
		Matrix3 inertia;

		void clear()
		{
			ray.reset();
			name = "";
			moveType = "";
			primType = "";
		}
	} mPickInfo;

	void pickClick(int x,int y,ClickMode mode);
	void pickOver(int x,int y);
	void pickLeave(int x,int y);
	bool getPickInfo(string &msg);

	// --------------------------------------------------------
	// 投擲

	void fire();

	// --------------------------------------------------------
	// 編集

	void deleteSelection();

	// --------------------------------------------------------
	// ワールド情報

	struct PfxAppWorldProperty {
		uint32_t maxInstances;			// 剛体インスタンスの最大数
		uint32_t maxDynBodies;			// 剛体の最大数
		uint32_t maxJoints;				// ジョイントの最大数
		uint32_t maxContactPairs;		// コンタクトペアの最大数
		uint32_t subStepCount;			// サブステップ
		uint32_t contactIteration;		// コンタクト演算の反復回数
		uint32_t jointIteration;		// ジョイント演算の反復回数
		uint32_t rigBufferSize;			// 剛体バッファサイズ
		uint32_t rayBufferSize;			// レイバッファサイズ
		uint32_t allocatedBuffSize;		// アロケート済みメモリ(MB)
		uint32_t pickPower;				// ピッキングジョイントのパワー
		uint32_t lineWidth;				// シルエット描画の線幅
		uint32_t frameRate;				// フレームレート
		bool sleepEnable;				// 
	} mWorldProperty;

	void getWorldProperty(PfxAppWorldProperty &wp);
	void setWorldProperty(const PfxAppWorldProperty &wp);

private:
	// --------------------------------------------------------
	// メッシュ

	map<uint32_t,ConvexMesh>	meshArray;
	map<uint32_t,LargeTriMesh>	largeMeshArray;

	// 描画用メッシュ
	// CollPrim::viData[2]に描画用メッシュのインデックスを格納
	vector<RenderUtil::Mesh>	renderMeshes;

	void convertToRenderMesh(RenderUtil::Mesh &rmesh,ConvexMesh &mesh);
	void convertToRenderMesh(RenderUtil::Mesh &rmesh,LargeTriMesh &mesh);

	// --------------------------------------------------------
	// Pfxファイル読込

private:
	PfxCommonImport mPfxReader;
	
	// 名前とインデックスのリンク
	map<string,uint32_t>		bodyIndexArray;
	map<string,uint32_t>		stateIndexArray;
	map<string,uint32_t>		jointIndexArray;
	map<uint32_t,string>		bodyNameArray;
	map<uint32_t,string>		stateNameArray;
	map<uint32_t,string>		jointNameArray;

	void convertWorld(const PfxRigWorldInfo &world);
	void convertBodies(const PfxRigBodyInfo &body);
	void convertStates(const PfxRigStateInfo &state);
	void convertJoints(const PfxRigJointInfo &joint);
	void convertNonContactPairs(const PfxRigNonContactPairInfo &pair);
	
	void createFlatMesh(RenderUtil::Mesh &rmesh,vector<float> &verts,vector<unsigned short> &indices);
	
	void createSceneFromFile();	// シーンを作成

public:
	void importSceneFromPfx(const char *filename);

	// --------------------------------------------------------
	// Snapshotファイル読込

private:
	PfxSnapshotReader mSnpReader;
	
	bool readSnapshot(const char *filename);

public:
	void importSceneFromSnapshot(const char *filename);
};

#endif /* __PFX_APP_H__ */

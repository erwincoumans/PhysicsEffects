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

#ifndef __COMMON_APP_H__
#define __COMMON_APP_H__

#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "../Render/GLRender.h"

class CommonApp
{
protected:
	// ---------------------------------------------------------------------------
	// Simulation Parameter

	bool mSimulate;

	// ---------------------------------------------------------------------------
	// GL Parameter

	GLRender *mRender;

	Matrix4 mMv;   // ModelView Matrix
	Matrix4 mProj; // Projection Matrix

public:

	float mLightRadius,mLightRadX,mLightRadY;
	Vector3 mLightPos;

	float mViewRadius,mViewRadX,mViewRadY,mViewHeight;
	Vector3	mViewPos;

	Vector3 mViewTarget;

public:
	CommonApp();
	virtual ~CommonApp();

	void setSimulate(bool b) {mSimulate = b;}
	bool getSimulate() {return mSimulate;}

	void setVSync(bool sw) {mRender->setVSync(sw);}

	// ---------------------------------------------------------------------------
	// Utility
	
	void getModelViewMatrix(Matrix4 &modelview);
	void getModelViewProjectionMatrix(Matrix4 &modelviewproj);
	void screenToWorld(Vector3 &screenPosition);
	void worldToScreen(Vector3 &worldPosition);

	virtual void createModelViewMatrix(Matrix4 &m);
	virtual void createProjectionMatrix(Matrix4 &m);

	// ---------------------------------------------------------------------------
	// Framework Method
	
	virtual bool onInit(GLRender *render);
	virtual void onShutdown();
	virtual bool onUpdate();
	virtual void onRender();
	virtual void onUserRender() {}
	virtual void onRenderAuxils() {}

	// ---------------------------------------------------------------------------
	// Physics Framework Method
	// ※ 物理エンジンに固有のAPIを継承先で記述してください

	// 物理エンジンの初期化と開放
	virtual bool onInitPhysicsEngine();
	virtual void onShutdownPhysicsEngine();

	// 物理ワールドの構築と破棄
	virtual void onInitPhysicsWorld();
	virtual void onReleasePhysicsWorld();

	// シーンの構築と破棄
	virtual void onInitPhysicsScene();
	virtual void onReleasePhysicsScene();

	// 物理ワールドをmTimeStepに指定された時間で進める
	virtual void onStartSimulation() {}
	virtual void onWaitSimulation() {}

	// シミュレーションの更新（剛体属性を変更するなど）
	virtual void onUpdateSimulation() {}

	// 物理ワールドの再構築 : 下記手順で呼ばれます
	//  onReleasePhysicsScene
	//  onReleasePhysicsWorld
	//  onShutdownPhysicsEngine
	//  onInitPhysicsEngine
	//  onInitPhysicsWorld
	//  onInitPhysicsScene
	virtual void onReset();

	// ---------------------------------------------------------------------------
	// Pick

	virtual void pickClick(int x,int y) {}
	virtual void pickOver(int x,int y) {}
	virtual void pickLeave(int x,int y) {}

	// ---------------------------------------------------------------------------
	// Import/Export

	virtual void importScene(const char *filename) {}
	virtual void exportScene(const char *filename) {}
};

#endif /* __COMMON_APP_H__ */

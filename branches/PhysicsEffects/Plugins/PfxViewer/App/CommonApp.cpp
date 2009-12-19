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

#include "CommonApp.h"

///////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor

CommonApp::CommonApp()
{
	mSimulate = false;

	mLightRadius = 40.0f;
	mLightRadX = -0.6f;
	mLightRadY = 0.6f;

	mViewRadius = 40.0f;
	mViewRadX = -0.01f;
	mViewRadY = 0.0f;
	mViewHeight = 0.0f;

	mViewTarget = Vector3(0.0f,mViewHeight,0.0f);
}

CommonApp::~CommonApp()
{
}

///////////////////////////////////////////////////////////////////////////////
// App Method

bool CommonApp::onInit(GLRender *render)
{
	if(!render->getActive())
		return false;

	mRender = render;

	glClearColor(0.0f,0.0f,0.0f, 0.0f);
	glClearDepth(1.0f);
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);

	onInitPhysicsEngine();
	onInitPhysicsWorld();
	onInitPhysicsScene();

	return true;
}

bool CommonApp::onUpdate()
{
	mLightPos = Matrix3::rotationY(mLightRadY) * Matrix3::rotationX(mLightRadX) * 
		Vector3(0,0,mLightRadius);

	mViewPos  =	Matrix3::rotationY(mViewRadY) * Matrix3::rotationX(mViewRadX) * 
		Vector3(0,0,mViewRadius);

	if(mSimulate) {
		onStartSimulation();
		onWaitSimulation();
		onUpdateSimulation();
	}

	return true;
}

void CommonApp::onRender()
{
	mRender->drawSceneBegin();
	
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDisable(GL_TEXTURE_2D);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	createProjectionMatrix(mProj);
	glMultMatrixf((GLfloat*)&mProj);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	createModelViewMatrix(mMv);
	glMultMatrixf((GLfloat*)&mMv);
	
	// drawObjects();
	
	onRenderAuxils();
	
	mRender->drawSceneEnd();
}

void CommonApp::onShutdown()
{
	glFinish();

	onReleasePhysicsScene();
	onReleasePhysicsWorld();
	onShutdownPhysicsEngine();
}

///////////////////////////////////////////////////////////////////////////////
// Simulation Method

bool CommonApp::onInitPhysicsEngine()
{
	return true;
}

void CommonApp::onShutdownPhysicsEngine()
{
}

void CommonApp::onInitPhysicsWorld()
{
}

void CommonApp::onReleasePhysicsWorld()
{
}

void CommonApp::onInitPhysicsScene()
{
}

void CommonApp::onReleasePhysicsScene()
{
}

void CommonApp::onReset()
{
	onReleasePhysicsScene();
	onReleasePhysicsWorld();
	onInitPhysicsWorld();
	onInitPhysicsScene();
}

///////////////////////////////////////////////////////////////////////////////
// Utility

void CommonApp::getModelViewMatrix(Matrix4 &modelview)
{
	modelview = mMv;
}

void CommonApp::getModelViewProjectionMatrix(Matrix4 &modelviewproj)
{
	modelviewproj = mProj * mMv;
}

void CommonApp::screenToWorld(Vector3 &screenPosition)
{
	Matrix4 mvp,mvpInv;
	getModelViewProjectionMatrix(mvp);
	mvpInv = inverse(mvp);

	Vector4 wp(screenPosition,1.0f);

	wp[0] /= (0.5f * mRender->getWidth());
	wp[1] /= (0.5f * mRender->getHeight());

	float w =	mvpInv[0][3] * wp[0] +  
				mvpInv[1][3] * wp[1] +  
				mvpInv[2][3] * wp[2] +  
				mvpInv[3][3];

	wp = mvpInv * wp;
	wp /= w;

	screenPosition = wp.getXYZ();
}

void CommonApp::worldToScreen(Vector3 &worldPosition)
{
	Vector4 sp(worldPosition,1.0f);

	Matrix4 mvp;
	getModelViewProjectionMatrix(mvp);

	sp = mvp * sp;
	sp /= sp[3];
	sp[0] *= (0.5f * mRender->getWidth());
	sp[1] *= (0.5f * mRender->getHeight());

	worldPosition = sp.getXYZ();
}

void CommonApp::createProjectionMatrix(Matrix4 &m)
{
	m = Matrix4::perspective(3.14f*35.0f/180.0f, mRender->getWidth()/mRender->getHeight(), 0.1f, 1000.0f);
}

void CommonApp::createModelViewMatrix(Matrix4 &m)
{
	m = Matrix4::lookAt(Point3(mViewTarget+mViewPos),Point3(mViewTarget),Vector3(0,1,0));
}

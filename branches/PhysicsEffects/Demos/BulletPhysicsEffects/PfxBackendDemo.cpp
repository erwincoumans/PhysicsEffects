/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#define USE_PHYSICSEFFECTS_SOLVER
#define USE_PARALLEL_DISPATCHER

#include "PfxBackendDemo.h"
#include "btPhysicsEffectsWorld.h"

#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h> //printf debugging


#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"
#include "BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.h"


#if defined (_WIN32)
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

#elif defined (USE_PTHREADS)

#include "BulletMultiThreaded/PosixThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

#else
//other platforms run the parallel code sequentially (until pthread support or other parallel implementation is added)
#include "BulletMultiThreaded/SequentialThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif //WIN32

class DemoApplication* createDemo()
{
	return new PfxBackendDemo();
}

void PfxBackendDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	glFlush();

	swapBuffers();

}


void PfxBackendDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}





void	PfxBackendDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(50.));

	
	btDefaultCollisionConstructionInfo configInfo;
	
	configInfo.m_customCollisionAlgorithmMaxElementSize = sizeof(SpuContactManifoldCollisionAlgorithm);
	
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration(configInfo);
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)

#ifdef USE_PARALLEL_DISPATCHER
	int maxNumOutstandingTasks = 4;//4;
#ifdef USE_WIN32_THREADING
	m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo("collision",processCollisionTask,	createCollisionLocalStoreMemory,maxNumOutstandingTasks));
#elif defined (USE_PTHREADS)
    PosixThreadSupport::ThreadConstructionInfo constructionInfo("collision",processCollisionTask,createCollisionLocalStoreMemory,maxNumOutstandingTasks);
    m_threadSupportCollision = new PosixThreadSupport(constructionInfo);
#else
	SequentialThreadSupport::SequentialThreadConstructionInfo colCI("collision",processCollisionTask,createCollisionLocalStoreMemory);
	SequentialThreadSupport* m_threadSupportCollision = new SequentialThreadSupport(colCI);
#endif //USE_WIN32_THREADING
	m_dispatcher = new	SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxNumOutstandingTasks,m_collisionConfiguration);
#else //USE_PARALLEL_DISPATCHER
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif




	m_dispatcher->setDispatcherFlags(0);
	



	//m_broadphase = new btDbvtBroadphase();
	m_broadphase = new btAxisSweep3(btVector3(-1000,-1000,-1000),btVector3(1000,1000,1000));

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

#if USE_PHYSICSEFFECTS_SOLVER
	m_dynamicsWorld = new btPhysicsEffectsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
#else
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld->getSolverInfo().m_numIterations = 4;
	//m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 64;//8192;
	int numMaxSpus = 1;
	
#endif

	m_dynamicsWorld->setForceUpdateAllAabbs(false);

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(250.),btScalar(50.),btScalar(250.)));
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);


		int size = 8;
		const float cubeSize = 1.0f;
		float spacing = cubeSize;
		btVector3 pos(0.0f, cubeSize * 2, 0.0f);
		float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;

		for(int k=0;k<47;k++) 
		{
			for(int j=0;j<size;j++) 
			{
				pos[2] = offset + (float)j * (cubeSize * 2.0f + spacing);
				for(int i=0;i<size;i++) {
					pos[0] = offset + (float)i * (cubeSize * 2.0f + spacing);

					btVector3 boxSize = btVector3(cubeSize,cubeSize,cubeSize);
					btTransform trans;
					trans.setIdentity();
					trans.setOrigin(pos);
					localCreateRigidBody(1,trans,colShape);
					
				}
			}
			offset -= 0.05f * spacing * (size-1);
			spacing *= 1.01f;
			pos[1] += (cubeSize * 2.0f + spacing);
		}
	

	}

	clientResetScene();
}
	

void	PfxBackendDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}





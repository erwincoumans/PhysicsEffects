/*
   Copyright (C) 2010 Sony Computer Entertainment Inc.
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

#include "btPhysicsEffectsWorld.h"
#include "LinearMath/btQuickprof.h"
#include "Physics/RigidBody/RigidBodies.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletMultiThreaded/Vectormath2Bullet.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"



#define HEAP_BYTES_RB (64*1024*1024)
unsigned char memPoolRB[HEAP_BYTES_RB] __attribute__ ((aligned(128)));
HeapManager gPoolRB(memPoolRB,HEAP_BYTES_RB);


btPhysicsEffectsWorld::btPhysicsEffectsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration)
:btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration)
{

	int numGatherScatterSpus = 5;

	m_rigidBodies = new RigidBodies(&gPoolRB);
	
	//arbitrary value, todo: add interface to broadphase
	int maxBodies = 2000;
	m_rigidBodies->worldProperty.maxInstances =maxBodies ;//pairCache->getMaxProxies();
	m_rigidBodies->worldProperty.maxDynBodies = maxBodies;//numActive?
	m_rigidBodies->worldProperty.maxContactPairs = maxBodies*10;
	m_rigidBodies->worldProperty.maxJoints = maxBodies * 2;


	m_rigidBodies->reset();

	//if no broadphase is passed, create a SPU broadphase
	
	if (pairCache)
	{
		m_ownsPairCache = false;
	} else
	{
		m_ownsPairCache = true;
//		btPEBroadphase* peBP = new btPEBroadphase(m_rigidBodies);
//		setBroadphase(peBP);
	}

	
	{
		void* mem = btAlignedAlloc(sizeof(btSimulationIslandManager),16);
		m_islandManager = new (mem) btSimulationIslandManager();
	}

	m_ownsIslandManager = true;


}
		
btPhysicsEffectsWorld::~btPhysicsEffectsWorld()
{



	if (m_ownsPairCache)
	{
		delete getBroadphase();
	}

	delete m_rigidBodies;
}

void	btPhysicsEffectsWorld::predictUnconstraintMotion(btScalar timeStep)
{
	btDiscreteDynamicsWorld::predictUnconstraintMotion( timeStep);
}


		
void	btPhysicsEffectsWorld::internalSingleStepSimulation( btScalar timeStep)
{

	BT_PROFILE("internalSingleStepSimulation");
	
	int numBodies = getNumCollisionObjects();
	int i;

	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	btDispatcherInfo& dispatchInfo = getDispatchInfo();

	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();

	///perform collision detection
	performDiscreteCollisionDetection();

	calculateSimulationIslands();

	m_islandManager->buildIslands(getCollisionWorld()->getDispatcher(),getCollisionWorld());

	{
		

		static bool onlyOnce = true;
		if (onlyOnce)
		{
			m_rigidBodies->worldProperty.maxDynBodies = numBodies;
			m_rigidBodies->worldProperty.maxInstances = numBodies;
			m_rigidBodies->worldProperty.maxJoints = getNumConstraints();
			onlyOnce = false;
			m_rigidBodies->reset();

		}

		{
			BT_PROFILE("gathering bodies");
			for (i=0;i<numBodies;i++)
			{
				btRigidBody* body = btRigidBody::upcast(this->getCollisionObjectArray()[i]);

				btVector3 invInertiaLocal = body->getInvInertiaDiagLocal();
				btMatrix3x3 invInertiaLocal3x3;

				invInertiaLocal3x3.setIdentity ();
				invInertiaLocal3x3[0][0] = invInertiaLocal[0];
				invInertiaLocal3x3[1][1] = invInertiaLocal[1];
				invInertiaLocal3x3[2][2] = invInertiaLocal[2];

				m_rigidBodies->bodies[i].setBodyInertiaInv(getVmMatrix3(invInertiaLocal3x3));
				m_rigidBodies->bodies[i].setMassInv(body->getInvMass());
				m_rigidBodies->bodies[i].setCollObject(0);
				m_rigidBodies->bodies[i].setElasticity(body->getRestitution());
				m_rigidBodies->bodies[i].setFriction(body->getFriction());

				m_rigidBodies->states[i].trbBodyIdx = i;
				m_rigidBodies->states[i].center[0] = 0.f;//body->getCenterOfMassPosition().getX();
				m_rigidBodies->states[i].center[1] = 0.f;//body->getCenterOfMassPosition().getY();
				m_rigidBodies->states[i].center[2] = 0.f;//body->getCenterOfMassPosition().getZ();

				m_rigidBodies->states[i].contactFilterSelf = 1;//body->getBroadphaseProxy()->m_collisionFilterGroup;
				m_rigidBodies->states[i].contactFilterTarget = 1;//body->getBroadphaseProxy()->m_collisionFilterMask;

				m_rigidBodies->states[i].half[0] = 1e30;//body->getCenterOfMassPosition().getX();
				m_rigidBodies->states[i].half[1] = 1e30;//body->getCenterOfMassPosition().getY();
				m_rigidBodies->states[i].half[2] = 1e30;//body->getCenterOfMassPosition().getZ();

				m_rigidBodies->states[i].fOmega[0] = body->getAngularVelocity().getX();
				m_rigidBodies->states[i].fOmega[1] = body->getAngularVelocity().getY();
				m_rigidBodies->states[i].fOmega[2] = body->getAngularVelocity().getZ();

				m_rigidBodies->states[i].fV[0] = body->getLinearVelocity().getX();
				m_rigidBodies->states[i].fV[1] = body->getLinearVelocity().getY();
				m_rigidBodies->states[i].fV[2] = body->getLinearVelocity().getZ();

				m_rigidBodies->states[i].fX[0] = body->getCenterOfMassPosition().getX();
				m_rigidBodies->states[i].fX[1] = body->getCenterOfMassPosition().getY();
				m_rigidBodies->states[i].fX[2] = body->getCenterOfMassPosition().getZ();

				m_rigidBodies->states[i].fQ[0] = body->getWorldTransform().getRotation().getX();
				m_rigidBodies->states[i].fQ[1] = body->getWorldTransform().getRotation().getY();
				m_rigidBodies->states[i].fQ[2] = body->getWorldTransform().getRotation().getZ();
				m_rigidBodies->states[i].fQ[3] = body->getWorldTransform().getRotation()[3];

				//m_rigidBodies->states[i].cl.clearAccum();

				body->setIslandTag(i);

				if (body->getInvMass())
				{
					m_rigidBodies->states[i].setMoveType(MoveTypeActive);
				}
				else
				{
					m_rigidBodies->states[i].setMoveType(MoveTypeFixed);
				}

			}
		}
	}

		

	{
		//copy over non-empty contact manifolds
		BT_PROFILE("build nonEmptyManifolds");
		m_nonEmptyManifolds.resize(0);

		int numManifolds = getDispatcher()->getNumManifolds();
		btPersistentManifold**	manifolds = getDispatcher()->getInternalManifoldPointer();

		//printf("numManifolds =%d\n",numManifolds);
		for (int i=0;i<numManifolds;i++)
		{
			btPersistentManifold* contactManifold = manifolds[i];
			if (contactManifold->getNumContacts())
			{
				btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
				btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
				if ((obA->isActive()) || (obB->isActive()))
					m_nonEmptyManifolds.push_back(contactManifold);
			}
		}
		//printf("m_nonEmptyManifolds.size()=%d\n",m_nonEmptyManifolds.size());
	}

	{
		

		//copy over all contact manifolds into Physics Effects
		int numManifolds = m_nonEmptyManifolds.size();
		this->m_rigidBodies->setContactCount(numManifolds);
		btPersistentManifold**	manifolds = &m_nonEmptyManifolds[0];

		int i=0;

		int numActualContactPairs = 0;

		{
			BT_PROFILE("gathering manifolds");
			
			for (;i<numManifolds;i++)
			{
				btPersistentManifold* contactManifold = manifolds[i];
				btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
				btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
			
				
				int numContacts = contactManifold->getNumContacts();
				if (!numContacts)
					continue;

				if ((!obA->isActive()) && (!obB->isActive()))
					continue;
		
				ContactPair &contact = m_rigidBodies->contactPairs[numActualContactPairs];//Pair(m_rigidBodies->sortedContactPairs[i])];
				contact.numContacts = numContacts;
				contact.stateIndex[0] = ((btRigidBody*)contactManifold->getBody0())->getIslandTag();
				contact.stateIndex[1] = ((btRigidBody*)contactManifold->getBody1())->getIslandTag();
				//setStatePair(this->m_rigidBodies->sortedContactPairs[i],contact.stateIndex[0],contact.stateIndex[1]);
				//setFlag(m_rigidBodies->sortedContactPairs[i],1);
				
				

				for (int j=0;j<numContacts;j++)
				{
					btManifoldPoint& pt = contactManifold->getContactPoint(j);
					ContactPoint &cp = contact.contactPoints[j];
					
					
					cp.distance = pt.getDistance();//sign?
					cp.duration = pt.getLifeTime();
					cp.localPointA[0] = pt.m_localPointA.getX();
					cp.localPointA[1] = pt.m_localPointA.getY();
					cp.localPointA[2] = pt.m_localPointA.getZ();
					ConstraintCache &constraint = cp.constraints[0];

					constraint.accumImpulse = pt.m_appliedImpulse;
					
				//	btVector3 p0,p1;
				//	btPlaneSpace1(pt.m_normalWorldOnB,p0,p1);
				//	cp.tangent = getVmVector3(p0);

					cp.localPointB[0] = pt.m_localPointB.getX();
					cp.localPointB[1] = pt.m_localPointB.getY();
					cp.localPointB[2] = pt.m_localPointB.getZ();

					//cp.worldPoint[0] = getVmVector3(pt.getPositionWorldOnA()) - getVmVector3(obA->getWorldTransform().getOrigin());
					//cp.worldPoint[1] = getVmVector3(pt.getPositionWorldOnB()) - getVmVector3(obB->getWorldTransform().getOrigin());
				
				//	pt.m_normalWorldOnB.normalize();
					
					cp.setNormal(getVmVector3(pt.m_normalWorldOnB));

					ConstraintCache &f1 = cp.constraints[1];
					f1.accumImpulse = 0.f;

					ConstraintCache &f2 = cp.constraints[2];
					f2.accumImpulse = 0.f;
					
					//cp.frictionDen = 0.f;
					//cp.impulseDen = 0.f;
					//cp.localVelocity[0] = Vectormath::Aos::Vector3(0.f,0.f,0.f);
					//cp.localVelocity[1] = Vectormath::Aos::Vector3(0.f,0.f,0.f);

					
				//	glBegin(GL_LINES);
				//	glColor3f(1, 0, 1);
					
				//	btVector3 ptA = pt.getPositionWorldOnA();
				//	btVector3 ptB = pt.getPositionWorldOnB();

				//	glVertex3d(ptA.x(),ptA.y(),ptA.z());
				//	glVertex3d(ptB.x(),ptB.y(),ptB.z());
				//	glEnd();
				}



				//ContactPair		*contactPairs		__attribute__ ((aligned(16)));
				//SortData		*sortedContactPairs	__attribute__ ((aligned(16)));

				//setBodyA(newPairs[numNewPairs],BodyId(aabbA));
				//setMovA(newPairs[numNewPairs],movA);

				
				setStateA(m_rigidBodies->sortedContactPairs[numActualContactPairs],contact.stateIndex[0]);
				setStateB(m_rigidBodies->sortedContactPairs[numActualContactPairs],contact.stateIndex[1]);
				setBodyA(m_rigidBodies->sortedContactPairs[numActualContactPairs],obA->getIslandTag());
				setBodyB(m_rigidBodies->sortedContactPairs[numActualContactPairs],obB->getIslandTag());
				//setMovA(newPairs[numActualContactPairs],movA);
				TrbState& stateA = m_rigidBodies->states[contact.stateIndex[0]];
				TrbState& stateB = m_rigidBodies->states[contact.stateIndex[1]];
				setMovA(m_rigidBodies->sortedContactPairs[numActualContactPairs],stateA.getMoveType());//movA???);
				setMovB(m_rigidBodies->sortedContactPairs[numActualContactPairs],stateB.getMoveType());//movA???);

				//setMovB(newPairs[numActualContactPairs],movB);
//				setMovType(m_rigidBodies->sortedContactPairs[numActualContactPairs],state.getMoveType());
				setPair(m_rigidBodies->sortedContactPairs[numActualContactPairs],numActualContactPairs);
				
				numActualContactPairs++;
			}
			
		}

		m_rigidBodies->setContactCount(numActualContactPairs);
		
		if(numActualContactPairs > 0) {
			//printf("numActualContactPairs=%d\n",numActualContactPairs);
			uint32_t numRemovedContactPairs  = 0;

			if (getDispatchInfo().m_enableSPU)
			{
				//numRemovedContactPairs = m_rigidBodies->refreshContactPairsSPU();
			}
			else
			{
				//numRemovedContactPairs = m_rigidBodies->refreshContactPairs();
				//m_rigidBodies->quickSort(m_rigidBodies->sortedContactPairs,0,numManifolds-1);
			}

			
			//numActualContactPairs -= numRemovedContactPairs;
		
		}
	}

	
	//m_rigidBodies->ppuSimulate(timeStep,0);

	
	


	{
		

		m_rigidBodies->setJointCount(0);
	//	int numInstances = 0;
		convertJoints();

		m_rigidBodies->setInstanceCount(numBodies);

		{
			BT_PROFILE("solving PPU");
			m_rigidBodies->solveConstraints(timeStep);
		}


	}

	{
		

		//m_rigidBodies->sleepOrWakeup();

		{
			BT_PROFILE("scattering velocities");
			for (i=0;i<numBodies;i++)
			{
				btRigidBody* body = btRigidBody::upcast(this->getCollisionObjectArray()[i]);
				if (body->getInvMass())
				{
					body->setLinearVelocity(btVector3(m_rigidBodies->states[i].fV[0],m_rigidBodies->states[i].fV[1],m_rigidBodies->states[i].fV[2]));
					body->setAngularVelocity(btVector3(m_rigidBodies->states[i].fOmega[0],m_rigidBodies->states[i].fOmega[1],m_rigidBodies->states[i].fOmega[2]));
				}
			}
		}
	}

	{
		
		int i=0;

		{
			BT_PROFILE("scattering applied impulses");
			int numActualContactPairs = 0;
			//copy over applied impulse
			int numManifolds = m_nonEmptyManifolds.size();

			for (;i<numManifolds;i++)
			{
				btPersistentManifold* contactManifold = m_nonEmptyManifolds[i];
				int numContacts = contactManifold->getNumContacts();
				if (!numContacts)
					continue;

				btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
				btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
			
				if (!obA->isActive()&&!obB->isActive())
					continue;
				
				ContactPair &contact = m_rigidBodies->contactPairs[numActualContactPairs];//Pair(m_rigidBodies->sortedContactPairs[i])];
			
				for (int j=0;j<numContacts;j++)
				{
					btManifoldPoint& pt = contactManifold->getContactPoint(j);
					ContactPoint &cp = contact.contactPoints[j];
					ConstraintCache &constraint = cp.constraints[0];
					pt.m_appliedImpulse = constraint.accumImpulse;
				}
				numActualContactPairs++;
			}
		}
	}

	///integrate transforms
	{
		BT_PROFILE("integrateTransforms");
		integrateTransforms(timeStep);
	}


	///update vehicle simulation
	updateVehicles(timeStep);

	updateActivationState( timeStep );

	if(0 != m_internalTickCallback) {
		(*m_internalTickCallback)(this, timeStep);
	}	

	

}

void	btPhysicsEffectsWorld::setNumTasks(int numTasks)
{
}



void	btPhysicsEffectsWorld::debugDrawWorld()
{
	btDiscreteDynamicsWorld::debugDrawWorld();

	
}


void btPhysicsEffectsWorld::convertJoints(void)
{
	int i, num_const;
	//copy joint constraints
	num_const = getNumConstraints();
	for(i = num_const - 1; i >= 0 ;i--)
	{
		btTypedConstraint* constraint = getConstraint(i);
		switch(constraint->getConstraintType())
		{
		case SLIDER_CONSTRAINT_TYPE:
			convertSliderJoint((btSliderConstraint*)constraint);
			break;
		case POINT2POINT_CONSTRAINT_TYPE:
			convertP2PJoint((btPoint2PointConstraint*)constraint);
			break;
		case HINGE_CONSTRAINT_TYPE:
			convertHingeJoint((btHingeConstraint*)constraint);
			break;
		case D6_CONSTRAINT_TYPE:
			convert6DofJoint((btGeneric6DofConstraint*) constraint);
			break;
		case CONETWIST_CONSTRAINT_TYPE:
			convertConeTwistJoint((btConeTwistConstraint*) constraint);
			break;
		default: 
			break;
		}
	}
	return;
} // btPhysicsEffectsWorld::convertJoints()

void btPhysicsEffectsWorld::convertSliderJoint(btSliderConstraint* pSlider)
{
#if 0
	const btRigidBody& rbA = pSlider->getRigidBodyA();
	const btRigidBody& rbB = pSlider->getRigidBodyB();
	int instanceA = rbA.getIslandTag();
	int instanceB = rbB.getIslandTag();
	pSlider->calculateTransforms();
	int newIdx = m_rigidBodies->createJoint();
	Joint* pJoint = m_rigidBodies->joints + newIdx;
	pJoint->reset();
	pJoint->maxIteration = m_rigidBodies->worldProperty.jointIteration;
	pJoint->jointType = JointTypeSlider;
	pJoint->linearDamping = pSlider->getDampingDirLin();
	pJoint->angularDamping = pSlider->getDampingDirAng();
	pJoint->maxLinearImpulse = 10.0F;
	pJoint->maxAngularImpulse = 10.0F;
	pJoint->linearImpulseWeight = 1.0F;
	pJoint->angularImpulseWeight = 1.0F;
	pJoint->linearErrorCorrection = 0.0F;
	pJoint->angularErrorCorrection = 0.0F;
	pJoint->targetFrame = Matrix3::identity();
	pJoint->stateIndexA = instanceA;
	pJoint->stateIndexB = instanceB;
	pJoint->breakableLimit = 0.0F;
	float lowLim = pSlider->getLowerLinLimit();
	float uppLim = pSlider->getUpperLinLimit();
	if(uppLim < lowLim)
	{
		pJoint->setFree(0);
	}
	else if(uppLim == lowLim)
	{
		pJoint->setLock(0);
	}
	else
	{
		pJoint->setLimit(0);
		pJoint->lowerLimit[0] = -0.5F * (uppLim - lowLim);
		pJoint->upperLimit[0] =  0.5F * (uppLim - lowLim);
	}
	pJoint->setLock(1);
	pJoint->setLock(2);
	lowLim = pSlider->getLowerAngLimit();
	uppLim = pSlider->getUpperAngLimit();
	if(uppLim < lowLim)
	{
		pJoint->setFree(3);
	}
	else if(uppLim == lowLim)
	{
		pJoint->setLock(3);
	}
	else
	{
		pJoint->setLimit(3);
		pJoint->lowerLimit[3] = lowLim;
		pJoint->upperLimit[3] = uppLim;
	}
	pJoint->setLock(4);
	pJoint->setLock(5);
	pJoint->anchorA = getVmVector3(pSlider->getAncorInA());
	pJoint->anchorB = getVmVector3(pSlider->getAncorInB());
	pJoint->frameA = getVmMatrix3(pSlider->getFrameOffsetA().getBasis());
	pJoint->frameB = getVmMatrix3(pSlider->getFrameOffsetB().getBasis());
	// motor
	if(pSlider->getPoweredLinMotor())
	{
		pJoint->motorFlag |= 1;
		pJoint->maxMotorForce[0] = pSlider->getMaxLinMotorForce();
		pJoint->targetMotorVel[0] = pSlider->getTargetLinMotorVelocity();
	}
	if(pSlider->getPoweredAngMotor())
	{
		pJoint->motorFlag |= (1 << 3);
		pJoint->maxMotorForce[3] = pSlider->getMaxAngMotorForce();
		pJoint->targetMotorVel[3] = pSlider->getTargetAngMotorVelocity();
	}
	return;
#endif
} // btPhysicsEffectsWorld::convertSliderJoint()

void btPhysicsEffectsWorld::convertP2PJoint(btPoint2PointConstraint* pP2PConst)
{
#if 0
	const btRigidBody& rbA = pP2PConst->getRigidBodyA();
	const btRigidBody& rbB = pP2PConst->getRigidBodyB();
	int instanceA = rbA.getIslandTag();
	int instanceB = rbB.getIslandTag();
	int newIdx = m_rigidBodies->createJoint();
	Joint* pJoint = m_rigidBodies->joints + newIdx;
	pJoint->reset();
	pJoint->maxIteration = m_rigidBodies->worldProperty.jointIteration;
	pJoint->jointType = JointTypeBall;
	pJoint->linearDamping = 0.0f;
	pJoint->angularDamping = 0.0f;
	pJoint->maxLinearImpulse = 10.0F;
	pJoint->maxAngularImpulse = 10.0F;
	pJoint->linearImpulseWeight = 1.0F;
	pJoint->angularImpulseWeight = 1.0F;
	pJoint->linearErrorCorrection = 0.0F;
	pJoint->angularErrorCorrection = 0.0F;
	pJoint->targetFrame = Matrix3::identity();
	pJoint->stateIndexA = instanceA;
	pJoint->stateIndexB = instanceB;
	pJoint->breakableLimit = 0.0F;
	pJoint->setLock(0);
	pJoint->setLock(1);
	pJoint->setLock(2);
	pJoint->setFree(3);
	pJoint->setFree(4);
	pJoint->setFree(5);
	pJoint->anchorA = getVmVector3(pP2PConst->getPivotInA());
	pJoint->anchorB = getVmVector3(pP2PConst->getPivotInB());
	pJoint->frameA = Matrix3::identity();
	pJoint->frameB = Matrix3::identity();
	return;
#endif

} // btPhysicsEffectsWorld::convertP2PJoint()



void btPhysicsEffectsWorld::convertHingeJoint(btHingeConstraint* pHinge)
{
#if 0
	const btRigidBody& rbA = pHinge->getRigidBodyA();
	const btRigidBody& rbB = pHinge->getRigidBodyB();
	int instanceA = rbA.getIslandTag();
	int instanceB = rbB.getIslandTag();
	int newIdx = m_rigidBodies->createJoint();
	Joint* pJoint = m_rigidBodies->joints + newIdx;
	pJoint->reset();
	pJoint->maxIteration = m_rigidBodies->worldProperty.jointIteration;
	pJoint->jointType = JointTypeHinge;
	pJoint->linearDamping = 0.f;
	pJoint->angularDamping = 0.f;
	pJoint->maxLinearImpulse = 10.0F;
	pJoint->maxAngularImpulse = 10.0F;
	pJoint->linearImpulseWeight = 1.0F;
	pJoint->angularImpulseWeight = 1.0F;
	pJoint->linearErrorCorrection = 0.F;
	pJoint->angularErrorCorrection = 0.F;
	pJoint->targetFrame = Matrix3::identity();
	pJoint->stateIndexA = instanceA;
	pJoint->stateIndexB = instanceB;
	pJoint->breakableLimit = 0.0F;
	if(pHinge->getAngularOnly())
	{
		pJoint->setFree(0);
		pJoint->setFree(1);
		pJoint->setFree(2);
	}
	else
	{
		pJoint->setLock(0);
		pJoint->setLock(1);
		pJoint->setLock(2);
	}
	pJoint->setLock(3);
	float uppLim = -pHinge->getLowerLimit(); // frameB in Bullet has flipped Z-axis by construction 
	float lowLim = -pHinge->getUpperLimit(); 
	if(uppLim < lowLim)
	{
		pJoint->setFree(4);
	}
	else if(uppLim == lowLim)
	{
		pJoint->setLock(4);
	}
	else
	{
		pJoint->setLimit(4);
		pJoint->lowerLimit[4] = lowLim;
		pJoint->upperLimit[4] = uppLim;
	}
	pJoint->setLock(5);
	pJoint->anchorA = getVmVector3(pHinge->getAFrame().getOrigin());
	pJoint->anchorB = getVmVector3(pHinge->getBFrame().getOrigin());

	pJoint->frameA = getVmMatrix3(pHinge->getAFrame().getBasis());
	pJoint->frameB = getVmMatrix3(pHinge->getBFrame().getBasis());
	Vector3 uaxis;
	uaxis = getVmVector3(pHinge->getBFrame().getBasis().getColumn(2));
	pJoint->frameB.setCol(2, -1.0F * uaxis); // frameB in Bullet has flipped Z-axis by construction 
	if(pHinge->getEnableAngularMotor())
	{
		pJoint->motorFlag |= (1 << 3);
		pJoint->maxMotorForce[4] = pHinge->getMaxMotorImpulse();
		pJoint->targetMotorVel[4] = pHinge->getMotorTargetVelosity();
	}
	return;
#endif

} // btPhysicsEffectsWorld::convertHingeJoint()


void btPhysicsEffectsWorld::convert6DofJoint(btGeneric6DofConstraint* p6Dof)
{
#if 0
	const btRigidBody& rbA = p6Dof->getRigidBodyA();
	const btRigidBody& rbB = p6Dof->getRigidBodyB();
	int instanceA = rbA.getIslandTag();
	int instanceB = rbB.getIslandTag();
	int newIdx = m_rigidBodies->createJoint();
	Joint* pJoint = m_rigidBodies->joints + newIdx;
	pJoint->reset();
	pJoint->maxIteration = m_rigidBodies->worldProperty.jointIteration;
	pJoint->jointType = JointType6DOF;
	pJoint->linearDamping = 0.0f;
	pJoint->angularDamping = 0.0f;
	pJoint->maxLinearImpulse = 10.0F;
	pJoint->maxAngularImpulse = 10.0F;
	pJoint->linearImpulseWeight = 1.0F;
	pJoint->angularImpulseWeight = 1.0F;
	pJoint->linearErrorCorrection = 0.0F;
	pJoint->angularErrorCorrection = 0.0F;
	pJoint->targetFrame = Matrix3::identity();
	pJoint->stateIndexA = instanceA;
	pJoint->stateIndexB = instanceB;
	pJoint->breakableLimit = 0.0F;
	// linear
	int i;
	for(i = 0; i < 3; i++)
	{
		float uppLim = p6Dof->getTranslationalLimitMotor()->m_upperLimit[i];
		float lowLim = p6Dof->getTranslationalLimitMotor()->m_lowerLimit[i];
		if(uppLim < lowLim)
		{
			pJoint->setFree(i);
		}
		else if(uppLim == lowLim)
		{
			pJoint->setLock(i);
		}
		else
		{
			pJoint->setLimit(i);
			pJoint->lowerLimit[i] = lowLim;
			pJoint->upperLimit[i] = uppLim;
		}
	}
	// angular
	for(i = 0; i < 3; i++)
	{
		// we define positive Euler angles using right screw rule (unlike btGeneric6DofConstraint)
		float uppLim = -p6Dof->getRotationalLimitMotor(i)->m_loLimit;
		float lowLim = -p6Dof->getRotationalLimitMotor(i)->m_hiLimit;
		if(uppLim < lowLim)
		{
			pJoint->setFree(i+3);
		}
		else if(uppLim == lowLim)
		{
			pJoint->setLock(i+3);
		}
		else
		{
			pJoint->setLimit(i+3);
			pJoint->lowerLimit[i+3] = lowLim;
			pJoint->upperLimit[i+3] = uppLim;
		}
		if(p6Dof->getRotationalLimitMotor(i)->m_enableMotor)
		{
			pJoint->motorFlag |= (1 << (i + 3));
			pJoint->maxMotorForce[i+3] = p6Dof->getRotationalLimitMotor(i)->m_maxMotorForce;
			pJoint->targetMotorVel[i+3] = p6Dof->getRotationalLimitMotor(i)->m_targetVelocity;
		}
	}
	pJoint->anchorA = getVmVector3(p6Dof->getFrameOffsetA().getOrigin());
	pJoint->anchorB = getVmVector3(p6Dof->getFrameOffsetB().getOrigin());
	pJoint->frameA = getVmMatrix3(p6Dof->getFrameOffsetA().getBasis());
	pJoint->frameB = getVmMatrix3(p6Dof->getFrameOffsetB().getBasis());
	return;
#endif
} // btPhysicsEffectsWorld::convert6DofJoint()


void btPhysicsEffectsWorld::convertConeTwistJoint(btConeTwistConstraint* pConeTwist)
{
#if 0
	const btRigidBody& rbA = pConeTwist->getRigidBodyA();
	const btRigidBody& rbB = pConeTwist->getRigidBodyB();
	int instanceA = rbA.getIslandTag();
	int instanceB = rbB.getIslandTag();
	int newIdx = m_rigidBodies->createJoint();
	Joint* pJoint = m_rigidBodies->joints + newIdx;
	pJoint->reset();
	pJoint->maxIteration = m_rigidBodies->worldProperty.jointIteration;
	pJoint->jointType = JointTypeConeTwist;
	pJoint->linearDamping = 0.0f;
	pJoint->angularDamping = 0.0f;
	pJoint->maxLinearImpulse = 10.0F;
	pJoint->maxAngularImpulse = 10.0F;
	pJoint->linearImpulseWeight = 1.0F;
	pJoint->angularImpulseWeight = 1.0F;
	pJoint->linearErrorCorrection = 0.0F;
	pJoint->angularErrorCorrection = 0.0F;
	pJoint->targetFrame = Matrix3::identity();
	pJoint->stateIndexA = instanceA;
	pJoint->stateIndexB = instanceB;
	pJoint->breakableLimit = 0.0F;
	pJoint->setLock(0);
	pJoint->setLock(1);
	pJoint->setLock(2);
	pJoint->setFree(3);
	pJoint->setFree(4);
	pJoint->setFree(5);
	pJoint->upperLimit[3] = pConeTwist->m_swingSpan1;
	pJoint->upperLimit[4] = pConeTwist->m_swingSpan2;
	pJoint->upperLimit[5] = pConeTwist->m_twistSpan;
	pJoint->anchorA = getVmVector3(pConeTwist->getAFrame().getOrigin());
	pJoint->anchorB = getVmVector3(pConeTwist->getBFrame().getOrigin());
	pJoint->frameA = getVmMatrix3(pConeTwist->getAFrame().getBasis());
	pJoint->frameB = getVmMatrix3(pConeTwist->getBFrame().getBasis());
	return;
#endif

} // btPhysicsEffectsWorld::convertConeTwistJoint()

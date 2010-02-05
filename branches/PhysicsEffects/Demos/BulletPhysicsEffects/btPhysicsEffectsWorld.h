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

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
//#define IN_PARALLELL_SOLVER 1 // to get access to private class members
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
//#undef IN_PARALLELL_SOLVER

#ifndef BT_PHYSICS_EFFECTS_WORLD_H
#define BT_PHYSICS_EFFECTS_WORLD_H

class RigidBodies;
class HeapManager;
class RigidBodyTaskMulti;

///btPhysicsEffectsWorld add Physics Effects implementation for Bullet
class btPhysicsEffectsWorld : public btDiscreteDynamicsWorld
{
		
	RigidBodies*	m_rigidBodies;

	bool			m_ownsPairCache;


protected:
	
	virtual void	predictUnconstraintMotion(btScalar timeStep);
	
	virtual void	internalSingleStepSimulation( btScalar timeStep);

	
	virtual void	debugDrawWorld();

	void convertJoints(void);
	void convertSliderJoint(btSliderConstraint* pSlider);
	void convertP2PJoint(btPoint2PointConstraint* pP2PConst);
	void convertHingeJoint(btHingeConstraint* pHinge);
	void convert6DofJoint(btGeneric6DofConstraint* p6Dof);
	void convertConeTwistJoint(btConeTwistConstraint* pConeTwist);

	btAlignedObjectArray<class btPersistentManifold*>	m_nonEmptyManifolds;

public:
	
	btPhysicsEffectsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration);

	virtual ~btPhysicsEffectsWorld();
		
	virtual void	setNumTasks(int numTasks);
			
		
};

#endif //BT_PHYSICS_EFFECTS_WORLD_H

/* [SCE CONFIDENTIAL DOCUMENT]
 * $PSLibId$
 *                Copyright (C) 2010 Sony Computer Entertainment Inc.
 *                                                All Rights Reserved.
 */
 #ifndef _BT_PARALLEL_DYNAMICS_WORLD_H
#define _BT_PARALLEL_DYNAMICS_WORLD_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "physics_effects/base_level/rigidbody/pfx_rigid_body.h"
#include "physics_effects/base_level/rigidbody/pfx_rigid_state.h"
#include "physics_effects/base_level/collision/pfx_collidable.h"
#include "physics_effects/base_level/solver/pfx_solver_body.h"

//#define USE_PE_GATHER_SCATTER_SPURS_TASK 1


class btThreadSupportInterface;
class SpuPEGatherScatterTaskProcess;
struct CellSpurs;
class SpuBatchRaycaster;
struct btLowLevelBroadphase;

#define PARALLEL_BATCH_SIZE 64


///btBulletPhysicsEffectsWorld adds parallel processing for integration/motion prediction
class btBulletPhysicsEffectsWorld : public btDiscreteDynamicsWorld
{

protected:
	btAlignedObjectArray<sce::PhysicsEffects::PfxRigidState>	m_lowLevelStates;
	btAlignedObjectArray<sce::PhysicsEffects::PfxRigidBody>		m_lowLevelBodies;
	btAlignedObjectArray<sce::PhysicsEffects::PfxSolverBody>	m_lowLevelSolverBodies;
	btAlignedObjectArray<sce::PhysicsEffects::PfxCollidable>	m_lowLevelCollidables;

	//PfxSolverBody solverBodies[NUM_RIGIDBODIES];

	class SpuPEGatherScatterTaskProcess*	m_PEGatherScatterProcess;

	struct btLowLevelData*							m_lowLevelData;

	void	createStateAndCollidable(btRigidBody* body);

	void	syncRigidBodyState(btRigidBody* body);
public:

	btBulletPhysicsEffectsWorld(struct btLowLevelData* lowLevelData, btDispatcher* dispatcher,btLowLevelBroadphase* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration, btThreadSupportInterface* threadSupport);
		
	virtual ~btBulletPhysicsEffectsWorld();
	
	virtual void	predictUnconstraintMotion(btScalar timeStep);
		
	virtual void integrateTransforms(btScalar timeStep);

	virtual void	solveConstraints(btContactSolverInfo& solverInfo);

	virtual void	calculateSimulationIslands();

	virtual void	addRigidBody(btRigidBody* body);

	virtual void	addRigidBody(btRigidBody* body, short group, short mask);

	virtual void	removeRigidBody(btRigidBody* body);


};
#endif //_BT_PARALLEL_DYNAMICS_WORLD_H
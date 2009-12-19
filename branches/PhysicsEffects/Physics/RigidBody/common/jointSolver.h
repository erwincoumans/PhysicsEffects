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
#ifndef __JOINT_SOLVER_H__
#define __JOINT_SOLVER_H__

#include "Physics/RigidBody/common/TrbStateVec.h"
#include "Physics/RigidBody/common/TrbDynBody.h"
#include "Physics/RigidBody/common/Joint.h"

struct JointSolverConfig {
	float timeStep;
	bool deformMeshEnable;
};

typedef void (*PreJoint)(Joint &joint,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const JointSolverConfig &config);

typedef void (*ApplyJoint)(Joint &joint,
		TrbState &stateA,
		TrbState &stateB,
		const JointSolverConfig &config);

void preJointFixAndMov(Joint &joint,
	TrbState &stateA,TrbDynBody &bodyA,
	TrbState &stateB,TrbDynBody &bodyB,
	const JointSolverConfig &config);

void preJointMovAndFix(Joint &joint,
	TrbState &stateA,TrbDynBody &bodyA,
	TrbState &stateB,TrbDynBody &bodyB,
	const JointSolverConfig &config);

void preJointMovAndMov(Joint &joint,
	TrbState &stateA,TrbDynBody &bodyA,
	TrbState &stateB,TrbDynBody &bodyB,
	const JointSolverConfig &config);

void preJointFixAndFix(Joint &joint,
	TrbState &stateA,TrbDynBody &bodyA,
	TrbState &stateB,TrbDynBody &bodyB,
	const JointSolverConfig &config);

void applyJointFixAndMov(Joint &joint,TrbState &stateA,TrbState &stateB,const JointSolverConfig &config);
void applyJointMovAndFix(Joint &joint,TrbState &stateA,TrbState &stateB,const JointSolverConfig &config);
void applyJointMovAndMov(Joint &joint,TrbState &stateA,TrbState &stateB,const JointSolverConfig &config);
void applyJointFixAndFix(Joint &joint,TrbState &stateA,TrbState &stateB,const JointSolverConfig &config);

#endif /* __JOINT_SOLVER_H__ */

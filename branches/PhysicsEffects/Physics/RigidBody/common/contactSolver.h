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

#ifndef __CONTACT_SOLVER_H__
#define __CONTACT_SOLVER_H__

#include "Physics/RigidBody/common/TrbStateVec.h"
#include "Physics/RigidBody/common/TrbDynBody.h"
#include "Physics/RigidBody/common/CollObject.h"
#include "Physics/RigidBody/common/Contact.h"

struct ContactSolverConfig {
	float timeStep;
	float separateBias;
	bool deformMeshEnable;
};

typedef void (*PreResponse)(ContactPair &pair,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const ContactSolverConfig &config);

typedef void (*ApplyImpulse)(ContactPair &pair,
		TrbState &stateA,
		TrbState &stateB,
		const ContactSolverConfig &config);

void preResponseFixAndMov(ContactPair &pair,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const ContactSolverConfig &config);

void preResponseMovAndFix(ContactPair &pair,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const ContactSolverConfig &config);

void preResponseMovAndMov(ContactPair &pair,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const ContactSolverConfig &config);

void preResponseFixAndFix(ContactPair &pair,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const ContactSolverConfig &config);

void applyImpulseFixAndMov(ContactPair &pair,TrbState &stateA,TrbState &stateB,const ContactSolverConfig &config);
void applyImpulseMovAndFix(ContactPair &pair,TrbState &stateA,TrbState &stateB,const ContactSolverConfig &config);
void applyImpulseMovAndMov(ContactPair &pair,TrbState &stateA,TrbState &stateB,const ContactSolverConfig &config);
void applyImpulseFixAndFix(ContactPair &pair,TrbState &stateA,TrbState &stateB,const ContactSolverConfig &config);

#endif /* __CONTACT_SOLVER_H__ */

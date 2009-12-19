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

#include "Physics/RigidBody/common/contactSolver.h"
#include "Physics/RigidBody/common/vec_utils.h"
#include "Physics/RigidBody/common/ConstraintRowSolver.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"

///////////////////////////////////////////////////////////////////////////////
// Pre Response

void preResponseFixAndFix(ContactPair &pair,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const ContactSolverConfig &config)
	{(void)pair;(void)stateA;(void)bodyA;(void)stateB;(void)bodyB;(void)config;}

void preResponseMovAndMov(ContactPair &pair,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const ContactSolverConfig &config)
{
	float restitution = 0.5f * (bodyA.getElasticity() + bodyB.getElasticity() );

	pair.compositeFriction = sqrtf(bodyA.getFriction() * bodyB.getFriction());

	Vector3 linVelA(stateA.getLinearVelocity());
	Vector3 linVelB(stateB.getLinearVelocity());
	Vector3 angVelA(stateA.getAngularVelocity());
	Vector3 angVelB(stateB.getAngularVelocity());

	Vector3 deltaLinVelA(stateA.getDeltaLinearVelocity());
	Vector3 deltaLinVelB(stateB.getDeltaLinearVelocity());
	Vector3 deltaAngVelA(stateA.getDeltaAngularVelocity());
	Vector3 deltaAngVelB(stateB.getDeltaAngularVelocity());

	Matrix3 mA(stateA.getOrientation());
	Matrix3 mB(stateB.getOrientation());
	
	float massInvA = bodyA.getMassInv();
	float massInvB = bodyB.getMassInv();
	Matrix3 inertiaInvA = mA * bodyA.getBodyInertiaInv() * transpose(mA);
	Matrix3 inertiaInvB = mB * bodyB.getBodyInertiaInv() * transpose(mB);

	for(int c=0;c<pair.numContacts;c++) {
		ContactPoint &cp = pair.contactPoints[c];

		Vector3 rA = mA * cp.getLocalPointA();
		Vector3 rB = mB * cp.getLocalPointB();

		Matrix3 K = Matrix3::scale(Vector3(massInvA + massInvB)) - 
				crossMatrix(rA) * inertiaInvA * crossMatrix(rA) - 
				crossMatrix(rB) * inertiaInvB * crossMatrix(rB);

		Vector3 vA = linVelA + cross(angVelA,rA);
		Vector3 vB = linVelB + cross(angVelB,rB);
		Vector3 vAB = vA-vB;

		Vector3 tangent1,tangent2;
		getPlaneSpace(cp.getNormal(),tangent1,tangent2);

		// Contact Constraint
		{
			ConstraintCache &constraint = cp.constraints[0];

			Vector3 normal = cp.getNormal();

			float denom = dot(K*normal,normal);

			constraint.rhs = -(1.0f+restitution)*dot(vAB,normal); // velocity error
			constraint.rhs -= (config.separateBias * PFX_MIN(0.0f,cp.distance+CONTACT_SLOP)) / config.timeStep; // position error
			constraint.rhs /= denom;
			constraint.jacDiagInv = (1.0f+restitution)/denom;
			constraint.lowerLimit = 0.0f;
			constraint.upperLimit = FLT_MAX;
			store_Vector3(normal,constraint.normal);

			float deltaImpulse = constraint.accumImpulse;
			deltaLinVelA += deltaImpulse * massInvA * normal;
			deltaAngVelA += deltaImpulse * inertiaInvA * cross(rA,normal);
			deltaLinVelB -= deltaImpulse * massInvB * normal;
			deltaAngVelB -= deltaImpulse * inertiaInvB * cross(rB,normal);
		}

		// Friction Constraint 1
		{
			ConstraintCache &constraint = cp.constraints[1];
			Vector3 normal = tangent1;

			float denom = dot(K*normal,normal);

			constraint.jacDiagInv = 1.0f/denom;
			constraint.rhs = -dot(vAB,normal);
			constraint.rhs *= constraint.jacDiagInv;
			constraint.lowerLimit = 0.0f;
			constraint.upperLimit = FLT_MAX;
			store_Vector3(normal,constraint.normal);

			float deltaImpulse = constraint.accumImpulse;
			deltaLinVelA += deltaImpulse * massInvA * normal;
			deltaAngVelA += deltaImpulse * inertiaInvA * cross(rA,normal);
			deltaLinVelB -= deltaImpulse * massInvB * normal;
			deltaAngVelB -= deltaImpulse * inertiaInvB * cross(rB,normal);
		}
		
		// Friction Constraint 2
		{
			ConstraintCache &constraint = cp.constraints[2];
			Vector3 normal = tangent2;

			float denom = dot(K*normal,normal);

			constraint.jacDiagInv = 1.0f/denom;
			constraint.rhs = -dot(vAB,normal);
			constraint.rhs *= constraint.jacDiagInv;
			constraint.lowerLimit = 0.0f;
			constraint.upperLimit = FLT_MAX;
			store_Vector3(normal,constraint.normal);

			float deltaImpulse = constraint.accumImpulse;
			deltaLinVelA += deltaImpulse * massInvA * normal;
			deltaAngVelA += deltaImpulse * inertiaInvA * cross(rA,normal);
			deltaLinVelB -= deltaImpulse * massInvB * normal;
			deltaAngVelB -= deltaImpulse * inertiaInvB * cross(rB,normal);
		}
	}

	stateA.setDeltaLinearVelocity(deltaLinVelA);
	stateB.setDeltaLinearVelocity(deltaLinVelB);
	stateA.setDeltaAngularVelocity(deltaAngVelA);
	stateB.setDeltaAngularVelocity(deltaAngVelB);

	pair.setMassInvA(massInvA);
	pair.setMassInvB(massInvB);
	pair.setInertiaInvA(inertiaInvA);
	pair.setInertiaInvB(inertiaInvB);
}

void preResponseMovAndFix(ContactPair &pair,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const ContactSolverConfig &config)
{
	float restitution = 0.5f * (bodyA.getElasticity() + bodyB.getElasticity() );

	pair.compositeFriction = sqrtf(bodyA.getFriction() * bodyB.getFriction());

	Vector3 linVelA(stateA.getLinearVelocity());
	Vector3 linVelB(stateB.getLinearVelocity());
	Vector3 angVelA(stateA.getAngularVelocity());
	Vector3 angVelB(stateB.getAngularVelocity());

	Vector3 deltaLinVelA(stateA.getDeltaLinearVelocity());
	Vector3 deltaAngVelA(stateA.getDeltaAngularVelocity());

	Matrix3 mA(stateA.getOrientation());
	Matrix3 mB(stateB.getOrientation());
	
	float massInvA = bodyA.getMassInv();
	Matrix3 inertiaInvA = mA * bodyA.getBodyInertiaInv() * transpose(mA);

	for(int c=0;c<pair.numContacts;c++) {
		ContactPoint &cp = pair.contactPoints[c];

		Vector3 rA = mA * cp.getLocalPointA();
		Vector3 rB = mB * cp.getLocalPointB();

		Matrix3 K = Matrix3::scale(Vector3(massInvA)) - 
				crossMatrix(rA) * inertiaInvA * crossMatrix(rA);

		Vector3 vA = linVelA + cross(angVelA,rA);
		Vector3 vB = linVelB + cross(angVelB,rB);
		if(PFX_UNLIKELY(config.deformMeshEnable && cp.subData.type == SubData::SubDataFacetLocal)) {
			vB += mB * cp.getLocalVelocityB();
		}
		Vector3 vAB = vA-vB;

		Vector3 tangent1,tangent2;
		getPlaneSpace(cp.getNormal(),tangent1,tangent2);

		// Contact Constraint
		{
			ConstraintCache &constraint = cp.constraints[0];

			Vector3 normal = cp.getNormal();

			float denom = dot(K*normal,normal);

			constraint.rhs = -(1.0f+restitution)*dot(vAB,normal); // velocity error
			constraint.rhs -= (config.separateBias * PFX_MIN(0.0f,cp.distance+CONTACT_SLOP)) / config.timeStep; // position error
			constraint.rhs /= denom;
			constraint.jacDiagInv = (1.0f+restitution)/denom;
			constraint.lowerLimit = 0.0f;
			constraint.upperLimit = FLT_MAX;
			store_Vector3(normal,constraint.normal);

			float deltaImpulse = constraint.accumImpulse;
			deltaLinVelA += deltaImpulse * massInvA * normal;
			deltaAngVelA += deltaImpulse * inertiaInvA * cross(rA,normal);
		}

		// Friction Constraint 1
		{
			ConstraintCache &constraint = cp.constraints[1];
			Vector3 normal = tangent1;

			float denom = dot(K*normal,normal);

			constraint.jacDiagInv = 1.0f/denom;
			constraint.rhs = -dot(vAB,normal);
			constraint.rhs *= constraint.jacDiagInv;
			constraint.lowerLimit = 0.0f;
			constraint.upperLimit = FLT_MAX;
			store_Vector3(normal,constraint.normal);

			float deltaImpulse = constraint.accumImpulse;
			deltaLinVelA += deltaImpulse * massInvA * normal;
			deltaAngVelA += deltaImpulse * inertiaInvA * cross(rA,normal);
		}
		
		// Friction Constraint 2
		{
			ConstraintCache &constraint = cp.constraints[2];
			Vector3 normal = tangent2;
			float denom = dot(K*normal,normal);

			constraint.jacDiagInv = 1.0f/denom;
			constraint.rhs = -dot(vAB,normal);
			constraint.rhs *= constraint.jacDiagInv;
			constraint.lowerLimit = 0.0f;
			constraint.upperLimit = FLT_MAX;
			store_Vector3(normal,constraint.normal);

			float deltaImpulse = constraint.accumImpulse;
			deltaLinVelA += deltaImpulse * massInvA * normal;
			deltaAngVelA += deltaImpulse * inertiaInvA * cross(rA,normal);
		}
	}
	stateA.setDeltaLinearVelocity(deltaLinVelA);
	stateA.setDeltaAngularVelocity(deltaAngVelA);
	pair.setMassInvA(massInvA);
	pair.setInertiaInvA(inertiaInvA);
}

void preResponseFixAndMov(ContactPair &pair,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const ContactSolverConfig &config)
{
	float restitution = 0.5f * (bodyA.getElasticity() + bodyB.getElasticity() );

	pair.compositeFriction = sqrtf(bodyA.getFriction() * bodyB.getFriction());

	Vector3 linVelA(stateA.getLinearVelocity());
	Vector3 linVelB(stateB.getLinearVelocity());
	Vector3 angVelA(stateA.getAngularVelocity());
	Vector3 angVelB(stateB.getAngularVelocity());

	Vector3 deltaLinVelB(stateB.getDeltaLinearVelocity());
	Vector3 deltaAngVelB(stateB.getDeltaAngularVelocity());

	Matrix3 mA(stateA.getOrientation());
	Matrix3 mB(stateB.getOrientation());
	
	float massInvB = bodyB.getMassInv();
	Matrix3 inertiaInvB = mB * bodyB.getBodyInertiaInv() * transpose(mB);

	for(int c=0;c<pair.numContacts;c++) {
		ContactPoint &cp = pair.contactPoints[c];

		Vector3 rA = mA * cp.getLocalPointA();
		Vector3 rB = mB * cp.getLocalPointB();

		Matrix3 K = Matrix3::scale(Vector3(massInvB)) - 
				crossMatrix(rB) * inertiaInvB * crossMatrix(rB);

		Vector3 vA = linVelA + cross(angVelA,rA);
		Vector3 vB = linVelB + cross(angVelB,rB);
		if(PFX_UNLIKELY(config.deformMeshEnable && cp.subData.type == SubData::SubDataFacetLocal)) {
			vA += mA * cp.getLocalVelocityA();
		}
		Vector3 vAB = vA-vB;

		Vector3 tangent1,tangent2;
		getPlaneSpace(cp.getNormal(),tangent1,tangent2);

		// Contact Constraint
		{
			ConstraintCache &constraint = cp.constraints[0];

			Vector3 normal = cp.getNormal();

			float denom = dot(K*normal,normal);

			constraint.rhs = -(1.0f+restitution)*dot(vAB,normal); // velocity error
			constraint.rhs -= (config.separateBias * PFX_MIN(0.0f,cp.distance+CONTACT_SLOP)) / config.timeStep; // position error
			constraint.rhs /= denom;
			constraint.jacDiagInv = (1.0f+restitution)/denom;
			constraint.lowerLimit = 0.0f;
			constraint.upperLimit = FLT_MAX;
			store_Vector3(normal,constraint.normal);

			float deltaImpulse = constraint.accumImpulse;
			deltaLinVelB -= deltaImpulse * massInvB * normal;
			deltaAngVelB -= deltaImpulse * inertiaInvB * cross(rB,normal);
		}

		// Friction Constraint 1
		{
			ConstraintCache &constraint = cp.constraints[1];
			Vector3 normal = tangent1;

			float denom = dot(K*normal,normal);

			constraint.jacDiagInv = 1.0f/denom;
			constraint.rhs = -dot(vAB,normal);
			constraint.rhs *= constraint.jacDiagInv;
			constraint.lowerLimit = 0.0f;
			constraint.upperLimit = FLT_MAX;
			store_Vector3(normal,constraint.normal);

			float deltaImpulse = constraint.accumImpulse;
			deltaLinVelB -= deltaImpulse * massInvB * normal;
			deltaAngVelB -= deltaImpulse * inertiaInvB * cross(rB,normal);
		}
		
		// Friction Constraint 2
		{
			ConstraintCache &constraint = cp.constraints[2];
			Vector3 normal = tangent2;

			float denom = dot(K*normal,normal);

			constraint.jacDiagInv = 1.0f/denom;
			constraint.rhs = -dot(vAB,normal);
			constraint.rhs *= constraint.jacDiagInv;
			constraint.lowerLimit = 0.0f;
			constraint.upperLimit = FLT_MAX;
			store_Vector3(normal,constraint.normal);

			float deltaImpulse = constraint.accumImpulse;
			deltaLinVelB -= deltaImpulse * massInvB * normal;
			deltaAngVelB -= deltaImpulse * inertiaInvB * cross(rB,normal);
		}
	}
	stateB.setDeltaLinearVelocity(deltaLinVelB);
	stateB.setDeltaAngularVelocity(deltaAngVelB);
	pair.setMassInvB(massInvB);
	pair.setInertiaInvB(inertiaInvB);
}

///////////////////////////////////////////////////////////////////////////////
// Apply Contact Impulse

void applyImpulseFixAndFix(ContactPair &pair,TrbState &stateA,TrbState &stateB,const ContactSolverConfig &config)
	{(void)pair;(void)stateA;(void)stateB;(void)config;}

void applyImpulseMovAndMov(ContactPair &pair,TrbState &stateA,TrbState &stateB,const ContactSolverConfig &config)
{
	(void)config;

	Matrix3 inertiaInvA(pair.getInertiaInvA());
	Matrix3 inertiaInvB(pair.getInertiaInvB());
	
	Quat qA(stateA.getOrientation());
	Quat qB(stateB.getOrientation());

	for(int c=0;c<pair.numContacts;c++) {
		ContactPoint &cp = pair.contactPoints[c];

		Vector3 rA = rotate(qA,cp.getLocalPointA());
		Vector3 rB = rotate(qB,cp.getLocalPointB());
		
		solveLinearConstraintRowMovAndMov(cp.constraints[0],
			stateA,pair.getMassInvA(),inertiaInvA,rA,
			stateB,pair.getMassInvB(),inertiaInvB,rB);
		
		// Update Friction Limit
		float mf = pair.compositeFriction*fabsf(cp.constraints[0].accumImpulse);
		cp.constraints[1].lowerLimit = -mf;
		cp.constraints[1].upperLimit =  mf;
		cp.constraints[2].lowerLimit = -mf;
		cp.constraints[2].upperLimit =  mf;

		solveLinearConstraintRowMovAndMov(cp.constraints[1],
			stateA,pair.getMassInvA(),inertiaInvA,rA,
			stateB,pair.getMassInvB(),inertiaInvB,rB);

		solveLinearConstraintRowMovAndMov(cp.constraints[2],
			stateA,pair.getMassInvA(),inertiaInvA,rA,
			stateB,pair.getMassInvB(),inertiaInvB,rB);
	}
}

void applyImpulseMovAndFix(ContactPair &pair,TrbState &stateA,TrbState &stateB,const ContactSolverConfig &config)
{
	(void)config;
	
	Matrix3 inertiaInvA(pair.getInertiaInvA());
	Matrix3 inertiaInvB(pair.getInertiaInvB());
	
	Quat qA(stateA.getOrientation());
	Quat qB(stateB.getOrientation());

	for(int c=0;c<pair.numContacts;c++) {
		ContactPoint &cp = pair.contactPoints[c];

		Vector3 rA = rotate(qA,cp.getLocalPointA());
		Vector3 rB = rotate(qB,cp.getLocalPointB());
		
		solveLinearConstraintRowMovAndFix(cp.constraints[0],
			stateA,pair.getMassInvA(),inertiaInvA,rA,
			stateB,pair.getMassInvB(),inertiaInvB,rB);
		
		// Update Friction Limit
		float mf = pair.compositeFriction*fabsf(cp.constraints[0].accumImpulse);
		cp.constraints[1].lowerLimit = -mf;
		cp.constraints[1].upperLimit =  mf;
		cp.constraints[2].lowerLimit = -mf;
		cp.constraints[2].upperLimit =  mf;

		solveLinearConstraintRowMovAndFix(cp.constraints[1],
			stateA,pair.getMassInvA(),inertiaInvA,rA,
			stateB,pair.getMassInvB(),inertiaInvB,rB);

		solveLinearConstraintRowMovAndFix(cp.constraints[2],
			stateA,pair.getMassInvA(),inertiaInvA,rA,
			stateB,pair.getMassInvB(),inertiaInvB,rB);
	}
}



void applyImpulseFixAndMov(ContactPair &pair,TrbState &stateA,TrbState &stateB,const ContactSolverConfig &config)
{
	(void)config;
	
	Matrix3 inertiaInvA(pair.getInertiaInvA());
	Matrix3 inertiaInvB(pair.getInertiaInvB());
	
	Quat qA(stateA.getOrientation());
	Quat qB(stateB.getOrientation());

	for(int c=0;c<pair.numContacts;c++) {
		ContactPoint &cp = pair.contactPoints[c];

		Vector3 rA = rotate(qA,cp.getLocalPointA());
		Vector3 rB = rotate(qB,cp.getLocalPointB());
		
		solveLinearConstraintRowFixAndMov(cp.constraints[0],
			stateA,pair.getMassInvA(),inertiaInvA,rA,
			stateB,pair.getMassInvB(),inertiaInvB,rB);
		
		// Update Friction Limit
		float mf = pair.compositeFriction*fabsf(cp.constraints[0].accumImpulse);
		cp.constraints[1].lowerLimit = -mf;
		cp.constraints[1].upperLimit =  mf;
		cp.constraints[2].lowerLimit = -mf;
		cp.constraints[2].upperLimit =  mf;

		solveLinearConstraintRowFixAndMov(cp.constraints[1],
			stateA,pair.getMassInvA(),inertiaInvA,rA,
			stateB,pair.getMassInvB(),inertiaInvB,rB);

		solveLinearConstraintRowFixAndMov(cp.constraints[2],
			stateA,pair.getMassInvA(),inertiaInvA,rA,
			stateB,pair.getMassInvB(),inertiaInvB,rB);
	}
}

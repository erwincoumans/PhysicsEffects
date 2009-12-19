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

#include <float.h>
#include "Physics/RigidBody/common/jointSolver.h"
#include "Physics/RigidBody/common/vec_utils.h"
#include "Physics/RigidBody/common/ConstraintRowSolver.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"

///////////////////////////////////////////////////////////////////////////////
// Calc Joint Angle

typedef void (*CalcJointAngle)(Matrix3 &worldFrameA,Matrix3 &worldFrameB,float *angle,Vector3 *axis);

void calcJointAngleStd(Matrix3 &worldFrameA,Matrix3 &worldFrameB,float *angle,Vector3 *axis);
void calcJointAngleHinge(Matrix3 &worldFrameA,Matrix3 &worldFrameB,float *angle,Vector3 *axis);
void calcJointAngleUniversal(Matrix3 &worldFrameA,Matrix3 &worldFrameB,float *angle,Vector3 *axis);

// ジョイントアングル計算
CalcJointAngle funcTbl_calcJointAngle[JointTypeCount] = {
	calcJointAngleStd,calcJointAngleStd,calcJointAngleStd,calcJointAngleHinge,
	calcJointAngleHinge,calcJointAngleUniversal,calcJointAngleStd,calcJointAngleStd,
};

void calcJointAngleStd(Matrix3 &worldFrameA,Matrix3 &worldFrameB,float *angle,Vector3 *axis)
{
	// フレームA座標系への変換マトリクス
	Matrix3 frameBA = transpose(worldFrameA) * worldFrameB;

	// クォータニオン回転をtwistとswingに分離
	Quat swing,twist,qBA(frameBA);
	swing = Quat::rotation(Vector3(1,0,0),frameBA.getCol0());
	twist = conj(swing) * qBA;

	if(dot(twist,Quat::rotationX(0.0f)) < 0.0f) {
		twist = -twist;
	}

	// それぞれの回転軸と回転角度を算出
	angleAxis(normalize(twist),angle[0],axis[0]);
	angleAxis(normalize(swing),angle[1],axis[1]);

	if(angle[1] < 0.00001f) {
		axis[1] = Vector3(0,1,0);
	}

	// twistの軸方向のチェック
	if(axis[0].getX() < 0.0f) {
		angle[0] = -angle[0];
	}

	axis[0] = worldFrameB.getCol0();
	axis[1] = worldFrameA * axis[1];
	axis[2] = cross(axis[0],axis[1]);
	angle[2] = 0.0f;
}

float atan2safe(float y,float x)
{
	if(x*x < 0.000001f || y*y < 0.000001f) {
		return 0.0f;
	}

	return atan2f(y,x);
}

void calcJointAngleHinge(Matrix3 &worldFrameA,Matrix3 &worldFrameB,float *angle,Vector3 *axis)
{
	Vector3 refA, refB, refC;
	float pa, pb;

	refA = worldFrameA.getCol1();
	refB = worldFrameA.getCol2();
	refC = worldFrameB.getCol1();
	pa = dot(refC, refA);
	pb = dot(refC, refB);

	axis[0] = worldFrameA.getCol0();
	angle[0] = atan2safe(pb, pa);

	refA = worldFrameA.getCol0();
	refB = worldFrameA.getCol1();
	refC = worldFrameB.getCol0();
	pa = dot(refC, refA);
	pb = dot(refC, refB);
	
	axis[1] = worldFrameA.getCol2();
	angle[1] = atan2safe(pb, pa);
	axis[2] = worldFrameA.getCol1();
	angle[2] = 0.0f;
}

void calcJointAngleUniversal(Matrix3 &worldFrameA,Matrix3 &worldFrameB,float *angle,Vector3 *axis)
{
	// フレームA座標系への変換マトリクス
	Matrix3 frameBA = transpose(worldFrameA) * worldFrameB;

	// クォータニオン回転をtwistとswingに分離
	Quat swing,swing1,swing2,twist,qBA(frameBA);
	Vector3 Pxy(frameBA.getCol0());
	Pxy[2] = 0.0f;
	swing1 = Quat::rotation(Vector3(1,0,0),normalize(Pxy));
	swing = Quat::rotation(Vector3(1,0,0),frameBA.getCol0());
	swing2 = swing * conj(swing1);
	twist = conj(swing) * qBA;

	if(dot(twist,Quat::rotationX(0.0f)) < 0.0f) {
		twist = -twist;
	}

	angleAxis(normalize(twist),angle[0],axis[0]);
	angleAxis(normalize(swing1),angle[1],axis[1]);
	angleAxis(normalize(swing2),angle[2],axis[2]);

	if(axis[1].getZ() < 0.0f) {
		axis[1] = -axis[1];
		angle[1] = -angle[1];
	}

	Vector3 chkY = cross(Vector3(0,0,1),frameBA.getCol0());
	if(dot(chkY,axis[2]) < 0.0f) {
		axis[2] = -axis[2];
		angle[2] = -angle[2];
	}

	// twistの軸方向のチェック
	if(axis[0].getX() < 0.0f) {
		angle[0] = -angle[0];
	}

	axis[0] = worldFrameB.getCol0();
	axis[1] = worldFrameA * axis[1];
	axis[2] = worldFrameA * axis[2];
}


///////////////////////////////////////////////////////////////////////////////
// Calc Joint Limit

static inline
void calcLinearLimit(Joint &joint,int c,float &posErr,float &velocityAmp,float &lowerLimit,float &upperLimit)
{
	if(joint.isFree(c)) {
		posErr = 0.0f;
		velocityAmp *= joint.linearDamping;
	}
	else if(joint.isLimit(c)) {
		if(posErr >= joint.lowerLimit[c] && posErr <= joint.upperLimit[c]) {
			posErr = 0.0f;
			velocityAmp *= joint.linearDamping;
		}
		else {
			if(posErr < joint.lowerLimit[c]) {
				posErr = posErr - joint.lowerLimit[c];
				posErr = PFX_MAX(0.0f,posErr-JOINT_LIN_SLOP);
				lowerLimit = PFX_MAX(0.0f,lowerLimit);
				velocityAmp = 1.0f;
			}
			else { // posErr > joint.upperLimit[c]
				posErr = posErr - joint.upperLimit[c];
				posErr = PFX_MIN(0.0f,posErr+JOINT_LIN_SLOP);
				upperLimit = PFX_MIN(0.0f,upperLimit);
				velocityAmp = 1.0f;
			}
		}
	}
}

static inline
void calcAngularLimit(Joint &joint,int c,float &posErr,float &velocityAmp,float &lowerLimit,float &upperLimit)
{
	if(joint.isFree(c)) {
		posErr = 0.0f;
		velocityAmp *= joint.angularDamping;
	}
	else if(joint.isLimit(c)) {
		if(posErr >= joint.lowerLimit[c] && posErr <= joint.upperLimit[c]) {
			posErr = 0.0f;
			velocityAmp *= joint.angularDamping;
		}
		else {
			if(posErr < joint.lowerLimit[c]) {
				posErr = posErr - joint.lowerLimit[c];
				posErr = PFX_MIN(0.0f,posErr+JOINT_ANG_SLOP);
				upperLimit = PFX_MIN(0.0f,upperLimit);
				velocityAmp = 1.0f;
			}
			else { // posErr > joint.upperLimit[c]
				posErr = posErr - joint.upperLimit[c];
				posErr = PFX_MAX(0.0f,posErr-JOINT_ANG_SLOP);
				lowerLimit = PFX_MAX(0.0f,lowerLimit);
				velocityAmp = 1.0f;
			}
		}
	}
	
	posErr = -posErr;
}

///////////////////////////////////////////////////////////////////////////////
// Pre Joint

void preJointFixAndFix(Joint &joint,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const JointSolverConfig &config)
	{(void)joint;(void)stateA;(void)bodyA;(void)stateB;(void)bodyB;(void)config;}

void preJointMovAndMov(Joint &joint,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const JointSolverConfig &config)
{
	Matrix3 mA(stateA.getOrientation());
	Matrix3 mB(stateB.getOrientation());
	
	Vector3 rA = rotate(stateA.getOrientation(),joint.anchorA);
	Vector3 rB = rotate(stateB.getOrientation(),joint.anchorB);

	Vector3 vA = stateA.getLinearVelocity() + cross(stateA.getAngularVelocity(),rA);
	Vector3 vB = stateB.getLinearVelocity() + cross(stateB.getAngularVelocity(),rB);
	Vector3 vAB = vA-vB;

	Vector3 distance = (stateA.getPosition() + rA) - (stateB.getPosition() + rB);

	Matrix3 worldFrameA,worldFrameB;
	worldFrameB = mB * joint.frameB;
	if(joint.jointType == JointTypeAnimation)
		worldFrameA = mA * joint.frameA * joint.targetFrame;
	else
		worldFrameA = mA * joint.frameA;

	float massInvA = bodyA.getMassInv();
	float massInvB = bodyB.getMassInv();
	Matrix3 inertiaInvA = mA * bodyA.getBodyInertiaInv() * transpose(mA);
	Matrix3 inertiaInvB = mB * bodyB.getBodyInertiaInv() * transpose(mB);
	
	// Linear 
	Matrix3 K = Matrix3::scale(Vector3(massInvA + massInvB)) - 
			crossMatrix(rA) * inertiaInvA * crossMatrix(rA) - 
			crossMatrix(rB) * inertiaInvB * crossMatrix(rB);

	int lcnt = (joint.jointType!=JointTypeDistance)?3:1;
	for(int c=0;c<lcnt;c++) {
		Vector3 normal = (joint.jointType!=JointTypeDistance)?-worldFrameA[c]:normalize((stateA.getPosition() + joint.rA) - (stateB.getPosition() + joint.rB));

		float posErr = dot(distance,normal);
		float lowerLimit = -joint.maxLinearImpulse;
		float upperLimit =  joint.maxLinearImpulse;
		float velocityAmp = 1.0f;
		
		calcLinearLimit(joint,c,posErr,velocityAmp,lowerLimit,upperLimit);

		float denom = dot(K*normal,normal);
		
		ConstraintCache &constraint = joint.constraints[c];
		
		constraint.rhs = -velocityAmp*dot(vAB,normal);
		constraint.rhs -= (joint.linearBias * posErr) / config.timeStep;
		constraint.rhs *= joint.linearImpulseWeight/denom;
		constraint.jacDiagInv = joint.linearImpulseWeight*velocityAmp/denom;
		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal,constraint.normal);

		if(joint.isWarmStarting(c)) {
			float deltaImpulse = constraint.accumImpulse;
			stateA.setDeltaLinearVelocity(stateA.getDeltaLinearVelocity() + deltaImpulse * massInvA * normal);
			stateA.setDeltaAngularVelocity(stateA.getDeltaAngularVelocity() + deltaImpulse * inertiaInvA * cross(rA,normal));
			stateB.setDeltaLinearVelocity(stateB.getDeltaLinearVelocity() - deltaImpulse * massInvB * normal);
			stateB.setDeltaAngularVelocity(stateB.getDeltaAngularVelocity() - deltaImpulse * inertiaInvB * cross(rB,normal));
		}
		else {
			constraint.accumImpulse = 0.0f;
		}
	}

	// ジョイント回転角と角度を計算
	Vector3 axis[3];
	float angle[3];
	funcTbl_calcJointAngle[joint.jointType](worldFrameA,worldFrameB,angle,axis);

	// Angular
	for(int c=0;c<3;c++) {
		Vector3 normal = axis[c];

		float posErr = angle[c];
		float lowerLimit = -joint.maxAngularImpulse;
		float upperLimit =  joint.maxAngularImpulse;
		float velocityAmp = 1.0f;
		
		calcAngularLimit(joint,c+3,posErr,velocityAmp,lowerLimit,upperLimit);

		float denom = dot((inertiaInvA+inertiaInvB)*normal,normal);
		
		ConstraintCache &constraint = joint.constraints[c+3];
		
		constraint.rhs = -velocityAmp*dot(stateA.getAngularVelocity()-stateB.getAngularVelocity(),normal); // velocity error
		constraint.rhs -= (joint.angularBias * posErr) / config.timeStep; // position error
		constraint.rhs *= joint.angularImpulseWeight/denom;
		constraint.jacDiagInv = joint.angularImpulseWeight*velocityAmp/denom;
		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal,constraint.normal);

		if(joint.isWarmStarting(c+3)) {
			float deltaImpulse = constraint.accumImpulse;
			stateA.setDeltaAngularVelocity(stateA.getDeltaAngularVelocity() + deltaImpulse * inertiaInvA * cross(rA,normal));
			stateB.setDeltaAngularVelocity(stateB.getDeltaAngularVelocity() - deltaImpulse * inertiaInvB * cross(rB,normal));
		}
		else {
			constraint.accumImpulse = 0.0f;
		}
	}

	joint.rA = rA;
	joint.rB = rB;
	joint.massInvA = massInvA;
	joint.inertiaInvA = inertiaInvA;
	joint.massInvB = massInvB;
	joint.inertiaInvB = inertiaInvB;
}

void preJointFixAndMov(Joint &joint,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const JointSolverConfig &config)
{
	(void) bodyA;

	Matrix3 mA(stateA.getOrientation());
	Matrix3 mB(stateB.getOrientation());
	
	Vector3 rA = rotate(stateA.getOrientation(),joint.anchorA);
	Vector3 rB = rotate(stateB.getOrientation(),joint.anchorB);

	Vector3 vA = stateA.getLinearVelocity() + cross(stateA.getAngularVelocity(),rA);
	Vector3 vB = stateB.getLinearVelocity() + cross(stateB.getAngularVelocity(),rB);
	if(PFX_UNLIKELY(config.deformMeshEnable && joint.subData.type == SubData::SubDataFacetLocal)) {
		vA += mA * joint.localVelocityA;
	}
	Vector3 vAB = vA-vB;

	Vector3 distance = (stateA.getPosition() + rA) - (stateB.getPosition() + rB);

	Matrix3 worldFrameA,worldFrameB;
	worldFrameB = mB * joint.frameB;
	if(joint.jointType == JointTypeAnimation)
		worldFrameA = mA * joint.frameA * joint.targetFrame;
	else
		worldFrameA = mA * joint.frameA;

	float massInvB = bodyB.getMassInv();
	Matrix3 inertiaInvB = mB * bodyB.getBodyInertiaInv() * transpose(mB);

	// Linear 
	Matrix3 K = Matrix3::scale(Vector3(massInvB)) - 
			crossMatrix(rB) * inertiaInvB * crossMatrix(rB);

	int lcnt = (joint.jointType!=JointTypeDistance)?3:1;
	for(int c=0;c<lcnt;c++) {
		Vector3 normal = (joint.jointType!=JointTypeDistance)?-worldFrameA[c]:normalize((stateA.getPosition() + joint.rA) - (stateB.getPosition() + joint.rB));

		float posErr = dot(distance,normal);
		float lowerLimit = -joint.maxLinearImpulse;
		float upperLimit =  joint.maxLinearImpulse;
		float velocityAmp = 1.0f;
		
		calcLinearLimit(joint,c,posErr,velocityAmp,lowerLimit,upperLimit);

		float denom = dot(K*normal,normal);
		
		ConstraintCache &constraint = joint.constraints[c];
		
		constraint.rhs = -velocityAmp*dot(vAB,normal);
		constraint.rhs -= (joint.linearBias * posErr) / config.timeStep;
		constraint.rhs *= joint.linearImpulseWeight/denom;
		constraint.jacDiagInv = joint.linearImpulseWeight*velocityAmp/denom;
		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal,constraint.normal);

		if(joint.isWarmStarting(c)) {
			float deltaImpulse = constraint.accumImpulse;
			stateB.setDeltaLinearVelocity(stateA.getDeltaLinearVelocity() - deltaImpulse * massInvB * normal);
			stateB.setDeltaAngularVelocity(stateB.getDeltaAngularVelocity() - deltaImpulse * inertiaInvB * cross(rB,normal));
		}
		else {
			constraint.accumImpulse = 0.0f;
		}
	}

	// ジョイント回転角と角度を計算
	Vector3 axis[3];
	float angle[3];
	funcTbl_calcJointAngle[joint.jointType](worldFrameA,worldFrameB,angle,axis);

	// Angular
	for(int c=0;c<3;c++) {
		Vector3 normal = axis[c];

		float posErr = angle[c];
		float lowerLimit = -joint.maxAngularImpulse;
		float upperLimit =  joint.maxAngularImpulse;
		float velocityAmp = 1.0f;
		
		calcAngularLimit(joint,c+3,posErr,velocityAmp,lowerLimit,upperLimit);

		float denom = dot(inertiaInvB*normal,normal);
		
		ConstraintCache &constraint = joint.constraints[c+3];
		
		constraint.rhs = -velocityAmp*dot(stateA.getAngularVelocity()-stateB.getAngularVelocity(),normal); // velocity error
		constraint.rhs -= (joint.angularBias * posErr) / config.timeStep; // position error
		constraint.rhs *= joint.angularImpulseWeight/denom;
		constraint.jacDiagInv = joint.angularImpulseWeight*velocityAmp/denom;
		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal,constraint.normal);

		if(joint.isWarmStarting(c+3)) {
			float deltaImpulse = constraint.accumImpulse;
			stateB.setDeltaAngularVelocity(stateA.getDeltaAngularVelocity() - deltaImpulse * inertiaInvB * cross(rB,normal));
		}
		else {
			constraint.accumImpulse = 0.0f;
		}
	}

	joint.rA = rA;
	joint.rB = rB;
	joint.massInvB = massInvB;
	joint.inertiaInvB = inertiaInvB;
}

void preJointMovAndFix(Joint &joint,
		TrbState &stateA,TrbDynBody &bodyA,
		TrbState &stateB,TrbDynBody &bodyB,
		const JointSolverConfig &config)
{
	(void) bodyB;

	Matrix3 mA(stateA.getOrientation());
	Matrix3 mB(stateB.getOrientation());
	
	Vector3 rA = rotate(stateA.getOrientation(),joint.anchorA);
	Vector3 rB = rotate(stateB.getOrientation(),joint.anchorB);

	Vector3 vA = stateA.getLinearVelocity() + cross(stateA.getAngularVelocity(),rA);
	Vector3 vB = stateB.getLinearVelocity() + cross(stateB.getAngularVelocity(),rB);
	if(PFX_UNLIKELY(config.deformMeshEnable && joint.subData.type == SubData::SubDataFacetLocal)) {
		vB += mB * joint.localVelocityB;
	}
	Vector3 vAB = vA-vB;

	Vector3 distance = (stateA.getPosition() + rA) - (stateB.getPosition() + rB);

	Matrix3 worldFrameA,worldFrameB;
	worldFrameB = mB * joint.frameB;
	if(joint.jointType == JointTypeAnimation)
		worldFrameA = mA * joint.frameA * joint.targetFrame;
	else
		worldFrameA = mA * joint.frameA;

	float massInvA = bodyA.getMassInv();
	Matrix3 inertiaInvA = mA * bodyA.getBodyInertiaInv() * transpose(mA);
	
	// Linear 
	Matrix3 K = Matrix3::scale(Vector3(massInvA)) - 
			crossMatrix(rA) * inertiaInvA * crossMatrix(rA);

	int lcnt = (joint.jointType!=JointTypeDistance)?3:1;
	for(int c=0;c<lcnt;c++) {
		Vector3 normal = (joint.jointType!=JointTypeDistance)?-worldFrameA[c]:normalize((stateA.getPosition() + joint.rA) - (stateB.getPosition() + joint.rB));

		float posErr = dot(distance,normal);
		float lowerLimit = -joint.maxLinearImpulse;
		float upperLimit =  joint.maxLinearImpulse;
		float velocityAmp = 1.0f;
		
		calcLinearLimit(joint,c,posErr,velocityAmp,lowerLimit,upperLimit);

		float denom = dot(K*normal,normal);
		
		ConstraintCache &constraint = joint.constraints[c];
		
		constraint.rhs = -velocityAmp*dot(vAB,normal);
		constraint.rhs -= (joint.linearBias * posErr) / config.timeStep;
		constraint.rhs *= joint.linearImpulseWeight/denom;
		constraint.jacDiagInv = joint.linearImpulseWeight*velocityAmp/denom;
		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal,constraint.normal);

		if(joint.isWarmStarting(c)) {
			float deltaImpulse = constraint.accumImpulse;
			stateA.setDeltaLinearVelocity(stateA.getDeltaLinearVelocity() + deltaImpulse * massInvA * normal);
			stateA.setDeltaAngularVelocity(stateA.getDeltaAngularVelocity() + deltaImpulse * inertiaInvA * cross(rA,normal));
		}
		else {
			constraint.accumImpulse = 0.0f;
		}
	}

	// ジョイント回転角と角度を計算
	Vector3 axis[3];
	float angle[3];
	funcTbl_calcJointAngle[joint.jointType](worldFrameA,worldFrameB,angle,axis);

	// Angular
	for(int c=0;c<3;c++) {
		Vector3 normal = axis[c];

		float posErr = angle[c];
		float lowerLimit = -joint.maxAngularImpulse;
		float upperLimit =  joint.maxAngularImpulse;
		float velocityAmp = 1.0f;
		
		calcAngularLimit(joint,c+3,posErr,velocityAmp,lowerLimit,upperLimit);

		float denom = dot(inertiaInvA*normal,normal);
		
		ConstraintCache &constraint = joint.constraints[c+3];
		
		constraint.rhs = -velocityAmp*dot(stateA.getAngularVelocity()-stateB.getAngularVelocity(),normal); // velocity error
		constraint.rhs -= (joint.angularBias * posErr) / config.timeStep; // position error
		constraint.rhs *= joint.angularImpulseWeight/denom;
		constraint.jacDiagInv = joint.angularImpulseWeight*velocityAmp/denom;
		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal,constraint.normal);

		if(joint.isWarmStarting(c+3)) {
			float deltaImpulse = constraint.accumImpulse;
			stateA.setDeltaAngularVelocity(stateA.getDeltaAngularVelocity() + deltaImpulse * inertiaInvA * cross(rA,normal));
		}
		else {
			constraint.accumImpulse = 0.0f;
		}
	}

	joint.rA = rA;
	joint.rB = rB;
	joint.massInvA = massInvA;
	joint.inertiaInvA = inertiaInvA;
}

///////////////////////////////////////////////////////////////////////////////
// Apply Joint Impulse

void applyJointFixAndFix(Joint &joint,TrbState &stateA,TrbState &stateB,const JointSolverConfig &config)
	{(void)joint;(void)stateA;(void)stateB;(void)config;}

void applyJointMovAndMov(Joint &joint,
		TrbState &stateA,
		TrbState &stateB,
		const JointSolverConfig &config)
{
	(void)config;
	
	float maxImpulse = -FLT_MAX;

	// Linear
	int lcnt = (joint.jointType!=JointTypeDistance)?3:1;
	for(int c=0;c<lcnt;c++) {
		solveLinearConstraintRowMovAndMov(joint.constraints[c],
			stateA,joint.massInvA,joint.inertiaInvA,joint.rA,
			stateB,joint.massInvB,joint.inertiaInvB,joint.rB);
		
		maxImpulse = PFX_MAX(maxImpulse,fabsf(joint.constraints[c].accumImpulse));
	}

	// Angular
	for(int c=0;c<3;c++) {
		solveAngularConstraintRowMovAndMov(joint.constraints[c+3],
			stateA,joint.inertiaInvA,
			stateB,joint.inertiaInvB);

		maxImpulse = PFX_MAX(maxImpulse,fabsf(joint.constraints[c+3].accumImpulse));
	}

	if(PFX_UNLIKELY(joint.isBreakable() && maxImpulse > joint.breakableLimit)) {
		joint.disableActive();
	}
}

// ジョイントインパルス計算
void applyJointFixAndMov(Joint &joint,
		TrbState &stateA,
		TrbState &stateB,
		const JointSolverConfig &config)
{
	(void)config;

	float maxImpulse = -FLT_MAX;

	// Linear
	int lcnt = (joint.jointType!=JointTypeDistance)?3:1;
	for(int c=0;c<lcnt;c++) {
		solveLinearConstraintRowFixAndMov(joint.constraints[c],
			stateA,joint.massInvA,joint.inertiaInvA,joint.rA,
			stateB,joint.massInvB,joint.inertiaInvB,joint.rB);

		maxImpulse = PFX_MAX(maxImpulse,fabsf(joint.constraints[c].accumImpulse));
	}

	// Angular
	for(int c=0;c<3;c++) {
		solveAngularConstraintRowFixAndMov(joint.constraints[c+3],
			stateA,joint.inertiaInvA,
			stateB,joint.inertiaInvB);

		maxImpulse = PFX_MAX(maxImpulse,fabsf(joint.constraints[c+3].accumImpulse));
	}
	
	if(PFX_UNLIKELY(joint.isBreakable() && maxImpulse > joint.breakableLimit)) {
		joint.disableActive();
	}
}

void applyJointMovAndFix(Joint &joint,
		TrbState &stateA,
		TrbState &stateB,
		const JointSolverConfig &config)
{
	(void)config;

	float maxImpulse = -FLT_MAX;

	// Linear
	int lcnt = (joint.jointType!=JointTypeDistance)?3:1;
	for(int c=0;c<lcnt;c++) {
		solveLinearConstraintRowMovAndFix(joint.constraints[c],
			stateA,joint.massInvA,joint.inertiaInvA,joint.rA,
			stateB,joint.massInvB,joint.inertiaInvB,joint.rB);

		maxImpulse = PFX_MAX(maxImpulse,fabsf(joint.constraints[c].accumImpulse));
	}

	// Angular
	for(int c=0;c<3;c++) {
		solveAngularConstraintRowMovAndFix(joint.constraints[c+3],
			stateA,joint.inertiaInvA,
			stateB,joint.inertiaInvB);

		maxImpulse = PFX_MAX(maxImpulse,fabsf(joint.constraints[c+3].accumImpulse));
	}
	
	if(PFX_UNLIKELY(joint.isBreakable() && maxImpulse > joint.breakableLimit)) {
		joint.disableActive();
	}
}

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

#include <algorithm>

#include "PfxApp.h"
#include "Physics/RigidBody/Mass.h"
#include "Util/MeshUtil.h"

///////////////////////////////////////////////////////////////////////////////
// 読込み

void PfxApp::createSceneFromFile()
{
	convertWorld(mPfxReader.getRigWorldInfo());
	
	for(int i=0;i<mPfxReader.getRigBodyCount();i++) {
		convertBodies(mPfxReader.getRigBodyById(i));
	}

	for(int i=0;i<mPfxReader.getRigStateCount();i++) {
		convertStates(mPfxReader.getRigStateById(i));
	}

	for(int i=0;i<mPfxReader.getRigJointCount();i++) {
		convertJoints(mPfxReader.getRigJointById(i));
	}
	
	for(int i=0;i<mPfxReader.getRigNonContactPairCount();i++) {
		convertNonContactPairs(mPfxReader.getRigNonContactPairById(i));
	}
}

///////////////////////////////////////////////////////////////////////////////
// enum変換

int getMoveType(PfxRigMoveType type)
{
	switch(type) {
		case RigMoveTypeFixed:
		return MoveTypeFixed;
		break;

		case RigMoveTypeActive:
		return MoveTypeActive;
		break;

		case RigMoveTypeKeyframe:
		return MoveTypeKeyframe;
		break;

		case RigMoveTypeOneWay:
		return MoveTypeOneWay;
		break;

		case RigMoveTypeTrigger:
		return MoveTypeTrigger;
		break;
	}

	return MoveTypeFixed;
}

int getJointType(PfxRigJointType type)
{
	switch(type) {
		case RigJointTypeBall:
		return JointTypeBall;
		break;

		case RigJointTypeChain:
		return JointTypeChain;
		break;

		case RigJointTypeSlider:
		return JointTypeSlider;
		break;

		case RigJointTypeHinge:
		return JointTypeHinge;
		break;

		case RigJointTypeFix:
		return JointTypeFix;
		break;

		case RigJointTypeUniversal:
		return JointTypeUniversal;
		break;

		case RigJointTypeAnimation:
		return JointTypeAnimation;
		break;

		case RigJointTypeDistance:
		return JointTypeDistance;
		break;
	}

	return JointTypeBall;
}

///////////////////////////////////////////////////////////////////////////////
// 描画用メッシュ作成

void PfxApp::createFlatMesh(RenderUtil::Mesh &rmesh,vector<float> &verts,vector<unsigned short> &indices)
{
	rmesh.numVerts = indices.size();
	rmesh.numIndices = indices.size();
	rmesh.vtx = new float[rmesh.numVerts*3];
	rmesh.nml = new float[rmesh.numVerts*3];
	rmesh.idx = new unsigned short[rmesh.numIndices];
	
	for(unsigned int i=0;i<indices.size()/3;i++) {
		Vector3 p0(verts[indices[i*3  ]*3  ],verts[indices[i*3  ]*3+1],verts[indices[i*3  ]*3+2]);
		Vector3 p1(verts[indices[i*3+1]*3  ],verts[indices[i*3+1]*3+1],verts[indices[i*3+1]*3+2]);
		Vector3 p2(verts[indices[i*3+2]*3  ],verts[indices[i*3+2]*3+1],verts[indices[i*3+2]*3+2]);
		
		Vector3 facetnml = normalize(cross(p2-p1,p0-p1));
		
		rmesh.idx[i*3  ] = i*3;
		rmesh.idx[i*3+1] = i*3+1;
		rmesh.idx[i*3+2] = i*3+2;

		rmesh.vtx[(i*3  )*3  ] = p0[0];
		rmesh.vtx[(i*3  )*3+1] = p0[1];
		rmesh.vtx[(i*3  )*3+2] = p0[2];
		rmesh.vtx[(i*3+1)*3  ] = p1[0];
		rmesh.vtx[(i*3+1)*3+1] = p1[1];
		rmesh.vtx[(i*3+1)*3+2] = p1[2];
		rmesh.vtx[(i*3+2)*3  ] = p2[0];
		rmesh.vtx[(i*3+2)*3+1] = p2[1];
		rmesh.vtx[(i*3+2)*3+2] = p2[2];

		rmesh.nml[(i*3  )*3  ] = facetnml[0];
		rmesh.nml[(i*3  )*3+1] = facetnml[1];
		rmesh.nml[(i*3  )*3+2] = facetnml[2];
		rmesh.nml[(i*3+1)*3  ] = facetnml[0];
		rmesh.nml[(i*3+1)*3+1] = facetnml[1];
		rmesh.nml[(i*3+1)*3+2] = facetnml[2];
		rmesh.nml[(i*3+2)*3  ] = facetnml[0];
		rmesh.nml[(i*3+2)*3+1] = facetnml[1];
		rmesh.nml[(i*3+2)*3+2] = facetnml[2];
	}
}

///////////////////////////////////////////////////////////////////////////////
// ワールドプロパティローダー

void PfxApp::convertWorld(const PfxRigWorldInfo &world)
{
	WorldProperty wp;
	wp.maxInstances = world.maxInstances;
	wp.maxDynBodies = world.maxDynBodies;
	wp.maxPrimitives = world.maxPrimitives;
	wp.maxJoints = world.maxJoints;
	wp.maxSprings = world.maxSprings;
	wp.maxContactPairs = world.maxContactPairs;
	wp.worldCenter = Vector3(world.worldCenter[0],world.worldCenter[1],world.worldCenter[2]);
	wp.worldExtent = Vector3(world.worldExtent[0],world.worldExtent[1],world.worldExtent[2]);
	wp.gravity = Vector3(world.gravity[0],world.gravity[1],world.gravity[2]);
	wp.subStepCount = world.subStepCount;
	wp.contactIteration = world.contactIteration;
	wp.jointIteration = world.jointIteration;
	wp.maxLinearVelocity = world.maxLinearVelocity;
	wp.maxAngularVelocity = world.maxAngularVelocity;
	wp.separateBias = world.separateBias;
	wp.sleepEnable = world.sleepEnable;
	wp.sleepCount = world.sleepCount;
	wp.sleepInterval = world.sleepInterval;
	wp.sleepLinearVelocity = world.sleepLinearVelocity;
	wp.sleepAngularVelocity = world.sleepAngularVelocity;
	wp.wakeLinearVelocity = world.wakeLinearVelocity;
	wp.wakeAngularVelocity = world.wakeAngularVelocity;
	wp.ccdEnable = world.ccdEnable;
	wp.deformMeshEnable = world.deformMeshEnable;

	int requiredBytes = checkRequiredMemory(wp);
	if(requiredBytes > mMemSize * 0.9f) {
		requiredBytes = requiredBytes * 1.1f;
		requiredBytes = ((requiredBytes + (1<<20)-1)>>20)<<20;
		mMemSize = requiredBytes;
		mWorldProperty.allocatedBuffSize = mMemSize>>20;
		onShutdownPhysicsEngine();
		onInitPhysicsEngine();
	}

	mRigidBodies->setWorldProperty(wp);
	mRigidBodies->reset();
}

void PfxApp::convertBodies(const PfxRigBodyInfo &body)
{
	CollObject *coll = mRigidBodies->createCollObject(body.shapes.size());
	assert(coll);

	RigidBodyProperty bodyProperty;
	bodyProperty.mass = body.mass;
	bodyProperty.inertia = Matrix3(
		Vector3(body.inertia[0][0],body.inertia[0][1],body.inertia[0][2]),
		Vector3(body.inertia[1][0],body.inertia[1][1],body.inertia[1][2]),
		Vector3(body.inertia[2][0],body.inertia[2][1],body.inertia[2][2]));
	bodyProperty.friction = body.friction;
	bodyProperty.restitution = body.restitution;
	bodyProperty.collObject = coll;
	TrbDynBody *dynBody = mRigidBodies->createRigidBody(bodyProperty);
	assert(dynBody);
	
	uint32_t bodyId = ((uint32_t)dynBody - (uint32_t)mRigidBodies->getTrbDynBodyByIndex(0))/sizeof(TrbDynBody);
	
	bodyIndexArray.insert(pair<string,uint32_t>(body.name,bodyId));
	bodyNameArray.insert(pair<uint32_t,string>(bodyId,body.name));
	
	for(int i=0;i<body.shapes.size();i++) {
		const PfxRigShapeInfo &shape = body.shapes[i];
		Transform3 trans(
			Quat(shape.relativeOrientation[0],shape.relativeOrientation[1],shape.relativeOrientation[2],shape.relativeOrientation[3]),
			Vector3(shape.relativePosition[0],shape.relativePosition[1],shape.relativePosition[2]));

		switch(shape.shapeType) {
			case RigShapeTypeSphere:
			coll->addSphere(Sphere(shape.vfData[0]),trans);
			break;

			case RigShapeTypeBox:
			coll->addBox(Box(shape.vfData[0],shape.vfData[1],shape.vfData[2]),trans);
			break;

			case RigShapeTypeCapsule:
			coll->addCapsule(Capsule(shape.vfData[0],shape.vfData[1]),trans);
			break;

			case RigShapeTypeConvexMesh:
			{
				map<uint32_t,ConvexMesh>::iterator i = meshArray.find(shape.viData[0]);
				if(i != meshArray.end()) {
					coll->addConvex(&i->second,trans);
				}
				else {
					PfxRigConvexMeshInfo *meshInfo = mPfxReader.getConvexMesh(shape.viData[0]);
					if(meshInfo) {
						ConvexMesh mesh;
						MeshUtil::createConvexMesh(mesh,
							(float*)&meshInfo->verts[0],meshInfo->verts.size()/3,
							(unsigned short*)&meshInfo->indices[0],meshInfo->indices.size());
						pair<map<uint32_t,ConvexMesh>::iterator, bool> check = meshArray.insert(
							pair<uint32_t,ConvexMesh>(meshInfo->id,mesh));
						if(check.second) {
							coll->addConvex(&check.first->second,trans);
							CollPrim &prim = coll->getPrim(coll->getNumPrims()-1);

							// 描画用メッシュ作成
							RenderUtil::Mesh rmesh;
							createFlatMesh(rmesh,meshInfo->verts,meshInfo->indices);
							renderMeshes.push_back(rmesh);
							prim.setPrimDataInteger(2,renderMeshes.size()-1);
						}
					}
					else {
						PRINTF("can't find mesh %d\n",shape.viData[0]);
					}
				}
			}
			break;

			case RigShapeTypeLargeMesh:
			{
				map<uint32_t,LargeTriMesh>::iterator i = largeMeshArray.find(shape.viData[0]);
				if(i != largeMeshArray.end()) {
					coll->setLargeMesh(&i->second);
				}
				else {
					PfxRigLargeMeshInfo *meshInfo = mPfxReader.getLargeMesh(shape.viData[0]);
					if(meshInfo) {
						LargeTriMesh mesh;
						MeshUtil::LargeTriMeshConfig config;
						config.numFacetsLimit = meshInfo->numFacetsLimit;
						config.islandsRatio = meshInfo->islandsRatio;
						MeshUtil::autoGenerateLargeTriMesh(config,mesh,
							(float*)&meshInfo->verts[0],meshInfo->verts.size()/3,
							(unsigned short*)&meshInfo->indices[0],meshInfo->indices.size());
						pair<map<uint32_t,LargeTriMesh>::iterator, bool> check = largeMeshArray.insert(
							pair<uint32_t,LargeTriMesh>(meshInfo->id,mesh));
						if(check.second) {
							coll->setLargeMesh(&check.first->second);
							CollPrim &prim = coll->getPrim(coll->getNumPrims()-1);

							// 描画用メッシュ作成
							RenderUtil::Mesh rmesh;
							createFlatMesh(rmesh,meshInfo->verts,meshInfo->indices);
							renderMeshes.push_back(rmesh);
							prim.setPrimDataInteger(2,renderMeshes.size()-1);
						}
						else {
							MeshUtil::releaseLargeTriMesh(mesh);
						}
					}
					else {
						PRINTF("can't find large mesh %d\n",shape.viData[0]);
					}
				}
			}
			break;

			default:
			break;
		};

		if(i < coll->getNumPrims()) {
			coll->getPrim(i).setContactFilterSelf(shape.contactFilterSelf);
			coll->getPrim(i).setContactFilterTarget(shape.contactFilterTarget);
		}
	}
	
	coll->finish();
}

void PfxApp::convertStates(const PfxRigStateInfo &state)
{
	InstanceProperty instProperty;
	instProperty.moveType = getMoveType(state.moveType);
	instProperty.contactFilterSelf = state.contactFilterSelf;
	instProperty.contactFilterTarget = state.contactFilterTarget;
	instProperty.position = Vector3(state.position[0],state.position[1],state.position[2]);
	instProperty.orientation = Quat(state.orientation[0],state.orientation[1],state.orientation[2],state.orientation[3]);
	instProperty.velocity = Vector3(state.linearVelocity[0],state.linearVelocity[1],state.linearVelocity[2]);
	instProperty.angularVelocity = Vector3(state.angularVelocity[0],state.angularVelocity[1],state.angularVelocity[2]);
	instProperty.linearDamping = state.linearDamping;
	instProperty.angularDamping = state.angularDamping;

	TrbDynBody *trbBody = NULL;
	map<string,uint32_t>::iterator p = bodyIndexArray.find(state.bodyName);
	if(p != bodyIndexArray.end()) {
		trbBody = mRigidBodies->getTrbDynBodyByIndex(p->second);
	}
	assert(trbBody);

	instProperty.rigidBody = trbBody;

	int instance = mRigidBodies->createInstance(instProperty);
	assert(instance >= 0);

	stateIndexArray.insert(pair<string,uint32_t>(state.name,instance));
	stateNameArray.insert(pair<uint32_t,string>(instance,state.name));

	TrbState *rigState = mRigidBodies->getState(instance);
	rigState->setUseCcd(state.useCcd);
	rigState->setUseSleep(state.useSleep);
	rigState->setUseContactCallback(state.useContactCallback);
	rigState->setUseSleepCallback(state.useSleepCallback);
	if(state.sleeping) {
		rigState->sleep();
	}
}

void PfxApp::convertJoints(const PfxRigJointInfo &joint)
{
	// ジョイント作成
	JointProperty jointProperty;
	jointProperty.jointType = getJointType(joint.jointType);
	
	map<string,uint32_t>::iterator pi = stateIndexArray.find(joint.parentName);
	map<string,uint32_t>::iterator ci = stateIndexArray.find(joint.childName);
	if(pi != stateIndexArray.end() && ci != stateIndexArray.end()) {
		jointProperty.parentBody = pi->second;
		jointProperty.childBody = ci->second;
	}
	else {
		return;
	}
	jointProperty.anchor = Vector3(joint.anchor[0],joint.anchor[1],joint.anchor[2]);
	jointProperty.axis = Vector3(joint.axis[0],joint.axis[1],joint.axis[2]);

	jointProperty.lowerLimit1 = joint.lowerLimit1;
	jointProperty.upperLimit1 = joint.upperLimit1;
	jointProperty.lowerLimit2 = joint.lowerLimit2;
	jointProperty.upperLimit2 = joint.upperLimit2;
	jointProperty.distance = joint.distance;

	jointProperty.linearDamping = joint.linearDamping;
	jointProperty.angularDamping = joint.angularDamping;
	jointProperty.linearImpulseWeight = joint.linearImpulseWeight;
	jointProperty.angularImpulseWeight = joint.angularImpulseWeight;
	jointProperty.linearBias = joint.linearBias;
	jointProperty.angularBias = joint.angularBias;
	jointProperty.maxLinearImpulse = joint.maxLinearImpulse;
	jointProperty.maxAngularImpulse = joint.maxAngularImpulse;
	jointProperty.breakableLimit = joint.breakableLimit;
	//jointProperty.warmStarting[0] = true;
	//jointProperty.warmStarting[1] = true;
	//jointProperty.warmStarting[2] = true;

	int j = mRigidBodies->createJoint(jointProperty);
	assert(j >= 0);

	jointIndexArray.insert(pair<string,uint32_t>(joint.name,j));
	jointNameArray.insert(pair<uint32_t,string>(j,joint.name));
}

void PfxApp::convertNonContactPairs(const PfxRigNonContactPairInfo &pair)
{
	map<string,uint32_t>::iterator iA = stateIndexArray.find(pair.rigA);
	map<string,uint32_t>::iterator iB = stateIndexArray.find(pair.rigB);
	
	if(iA != stateIndexArray.end() && iB != stateIndexArray.end()) {
		mRigidBodies->appendNonContactPair(iA->second,iB->second);
	}
}

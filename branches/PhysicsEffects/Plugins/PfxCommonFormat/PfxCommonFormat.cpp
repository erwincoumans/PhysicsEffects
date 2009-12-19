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

#include "PfxCommonFormat.h"
#include "Physics/RigidBody/Mass.h"

PfxMatrix3 PfxCommonFormat::calcTotalInertia(PfxRigBodyInfo &body,float scale)
{
	Matrix3 inertia(0.0f);

	if(body.shapes.empty()) return PfxMatrix3(0.0f);
	
	float mass = body.mass/(float)body.shapes.size();

	for(PfxUInt32 i=0;i<body.shapes.size();i++) {
		const PfxRigShapeInfo &shape = body.shapes[i];
		Vector3 offsetPos(shape.relativePosition[0],shape.relativePosition[1],shape.relativePosition[2]);
		Quat offsetRot(shape.relativeOrientation[0],shape.relativeOrientation[1],shape.relativeOrientation[2],shape.relativeOrientation[3]);
		Transform3 trans(offsetRot,offsetPos);
		Matrix3 I(0.0f);

		switch(shape.shapeType) {
			case RigShapeTypeSphere:
			calcInertiaSphere(shape.vfData[0],mass,I);
			break;

			case RigShapeTypeBox:
			calcInertiaBox(Vector3(shape.vfData[0],shape.vfData[1],shape.vfData[2]),mass,I);
			break;

			case RigShapeTypeCapsule:
			calcInertiaCylinderX(shape.vfData[1],shape.vfData[0]+shape.vfData[1],mass,I);
			break;

			case RigShapeTypeConvexMesh:
			{
				PfxRigConvexMeshInfo *meshInfo = getConvexMesh(shape.viData[0]);
				if(meshInfo) {
					calcInertiaMesh((float*)&meshInfo->verts[0],meshInfo->verts.size()/3,(unsigned short*)&meshInfo->indices[0],meshInfo->indices.size(),mass,I);
				}
			}
			break;

			default:
			break;
		}
		massRotate(I,Matrix3(offsetRot));
		massTranslate(mass,I,offsetPos);
		inertia += I;
	}
	
	inertia *= scale;

	return PfxMatrix3(inertia[0][0],inertia[0][1],inertia[0][2],
					  inertia[1][0],inertia[1][1],inertia[1][2],
					  inertia[2][0],inertia[2][1],inertia[2][2]);
}

void PfxCommonFormat::clear()
{
	mRigBodies.clear();
	mRigStates.clear();
	mRigJoints.clear();
	mRigConvexMeshes.clear();
	mRigLargeMeshes.clear();
	mRigNonContactPairs.clear();
}

void PfxCommonFormat::setRigWorldInfo(PfxRigWorldInfo &info)
{
	mRigWorldInfo = info;
}

PfxRigWorldInfo PfxCommonFormat::getRigWorldInfo()
{
	return mRigWorldInfo;
}

void PfxCommonFormat::addRigBody(PfxRigBodyInfo &body)
{
	int i=mRigBodies.size();
	mRigBodies.push_back(body);
	mRigBodyMap.insert(pair<string,PfxUInt32>(body.name,i));
}

void PfxCommonFormat::addRigState(PfxRigStateInfo &state)
{
	int i=mRigBodies.size();
	mRigStates.push_back(state);
	mRigStateMap.insert(pair<string,PfxUInt32>(state.name,i));
}

void PfxCommonFormat::addRigJoint(PfxRigJointInfo &joint)
{
	int i=mRigJoints.size();
	mRigJoints.push_back(joint);
	mRigJointMap.insert(pair<string,PfxUInt32>(joint.name,i));
}

void PfxCommonFormat::addRigConvexMesh(PfxRigConvexMeshInfo &convex)
{
	int i=mRigConvexMeshes.size();
	mRigConvexMeshes.push_back(convex);
	mRigConvexMeshMap.insert(pair<PfxUInt32,PfxUInt32>(convex.id,i));
}

void PfxCommonFormat::addRigLargeMesh(PfxRigLargeMeshInfo &largemesh)
{
	int i=mRigLargeMeshes.size();
	mRigLargeMeshes.push_back(largemesh);
	mRigLargeMeshMap.insert(pair<PfxUInt32,PfxUInt32>(largemesh.id,i));
}

void PfxCommonFormat::addRigNonContactPair(PfxRigNonContactPairInfo &info)
{
	mRigNonContactPairs.push_back(info);
}

int PfxCommonFormat::getRigBodyCount()
{
	return mRigBodies.size();
}

int PfxCommonFormat::getRigStateCount()
{
	return mRigStates.size();
}

int PfxCommonFormat::getRigJointCount()
{
	return mRigJoints.size();
}

int PfxCommonFormat::getConvexMeshCount()
{
	return mRigConvexMeshes.size();
}

int PfxCommonFormat::getLargeMeshCount()
{
	return mRigLargeMeshes.size();
}

int PfxCommonFormat::getRigNonContactPairCount()
{
	return mRigNonContactPairs.size();
}

const PfxRigBodyInfo       &PfxCommonFormat::getRigBodyById(int i) const
{
	return mRigBodies[i];
}

const PfxRigStateInfo      &PfxCommonFormat::getRigStateById(int i) const
{
	return mRigStates[i];
}

const PfxRigJointInfo      &PfxCommonFormat::getRigJointById(int i) const
{
	return mRigJoints[i];
}

const PfxRigConvexMeshInfo &PfxCommonFormat::getConvexMeshById(int i) const
{
	return mRigConvexMeshes[i];
}

const PfxRigLargeMeshInfo  &PfxCommonFormat::getLargeMeshById(int i) const
{
	return mRigLargeMeshes[i];
}

const PfxRigNonContactPairInfo  &PfxCommonFormat::getRigNonContactPairById(int i) const
{
	return mRigNonContactPairs[i];
}

PfxRigBodyInfo *PfxCommonFormat::getRigBody(string name)
{
	map<string,PfxUInt32>::iterator i;
	i = mRigBodyMap.find(name);
	if(i != mRigBodyMap.end()) {
		return &mRigBodies[i->second];
	}
	return NULL;
}

PfxRigStateInfo *PfxCommonFormat::getRigState(string name)
{
	map<string,PfxUInt32>::iterator i;
	i = mRigStateMap.find(name);
	if(i != mRigStateMap.end()) {
		return &mRigStates[i->second];
	}
	return NULL;
}

PfxRigJointInfo *PfxCommonFormat::getRigJoint(string name)
{
	map<string,PfxUInt32>::iterator i;
	i = mRigJointMap.find(name);
	if(i != mRigJointMap.end()) {
		return &mRigJoints[i->second];
	}
	return NULL;
}

PfxRigConvexMeshInfo *PfxCommonFormat::getConvexMesh(PfxUInt32 id)
{
	map<PfxUInt32,PfxUInt32>::iterator i;
	i = mRigConvexMeshMap.find(id);
	if(i != mRigConvexMeshMap.end()) {
		return &mRigConvexMeshes[i->second];
	}
	return NULL;
}

PfxRigLargeMeshInfo *PfxCommonFormat::getLargeMesh(PfxUInt32 id)
{
	map<PfxUInt32,PfxUInt32>::iterator i;
	i = mRigLargeMeshMap.find(id);
	if(i != mRigLargeMeshMap.end()) {
		return &mRigLargeMeshes[i->second];
	}
	return NULL;
}

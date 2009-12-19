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

#include "PfxCommonImport.h"

#define IMPORT_STRING_MAX 256

#define READ_STRING(tag,param)	else if(isSameTag(tag)) {char str[IMPORT_STRING_MAX];readString(str,IMPORT_STRING_MAX);param=str;}
#define READ_BOOL(tag,param)	else if(isSameTag(tag)) {readBool(param);}
#define READ_UINT32(tag,param)	else if(isSameTag(tag)) {readUInt32(param);}
#define READ_UINT16(tag,param)	else if(isSameTag(tag)) {PfxUInt32 tmp;readUInt32(tmp);param=(PfxUInt16)tmp;}
#define READ_UINT8(tag,param)	else if(isSameTag(tag)) {PfxUInt32 tmp;readUInt32(tmp);param=(PfxUInt8)tmp;}
#define READ_FLOAT(tag,param)	else if(isSameTag(tag)) {readFloat(param);}
#define READ_VEC3(tag,param)	else if(isSameTag(tag)) {readFloat3(param);}
#define READ_QUAT(tag,param)	else if(isSameTag(tag)) {readFloat4(param);}
#define READ_MATRIX3(tag,param)	else if(isSameTag(tag)) {readMatrix3(param);}
#define READ_FLOAT3(tag,param)	else if(isSameTag(tag)) {readFloat3(param[0],param[1],param[2]);}
#define READ_INDEX3(tag,param)	else if(isSameTag(tag)) {readIndex3(param[0],param[1],param[2]);}

PfxBool PfxCommonImport::readRigWorld(PfxRigWorldInfo &world)
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return true;
			}
		}
		READ_UINT32("maxInstances",world.maxInstances)
		READ_UINT32("maxDynBodies",world.maxDynBodies)
		READ_UINT32("maxPrimitives",world.maxPrimitives)
		READ_UINT32("maxJoints",world.maxJoints)
		READ_UINT32("maxSprings",world.maxSprings)
		READ_UINT32("maxContactPairs",world.maxContactPairs)
		READ_VEC3("worldExtent",world.worldExtent)
		READ_VEC3("worldCenter",world.worldCenter)
		READ_VEC3("gravity",world.gravity)
		READ_UINT8("subStepCount",world.subStepCount)
		READ_UINT8("contactIteration",world.contactIteration)
		READ_UINT8("jointIteration",world.jointIteration)
		READ_FLOAT("maxLinearVelocity",world.maxLinearVelocity)
		READ_FLOAT("maxAngularVelocity",world.maxAngularVelocity)
		READ_FLOAT("separateBias",world.separateBias)
		READ_BOOL("sleepEnable",world.sleepEnable)
		READ_UINT16("sleepInterval",world.sleepInterval)
		READ_UINT16("sleepCount",world.sleepCount)
		READ_FLOAT("sleepLinearVelocity",world.sleepLinearVelocity)
		READ_FLOAT("sleepAngularVelocity",world.sleepAngularVelocity)
		READ_FLOAT("wakeLinearVelocity",world.wakeLinearVelocity)
		READ_FLOAT("wakeAngularVelocity",world.wakeAngularVelocity)
		READ_BOOL("ccdEnable",world.ccdEnable)
		READ_BOOL("deformMeshEnable",world.deformMeshEnable)
	}
	return false;
}

void PfxCommonImport::readRigBodies()
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return;
			}
		}
		else if(isSameTag("RigBody")) {
			PfxRigBodyInfo body;
			if(readRigBody(body)) {
				addRigBody(body);
			}
		}
	}
}

void PfxCommonImport::readRigStates()
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return;
			}
		}
		else if(isSameTag("RigState")) {
			PfxRigStateInfo state;
			if(readRigState(state)) {
				addRigState(state);
			}
		}
	}
}

void PfxCommonImport::readRigJoints()
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return;
			}
		}
		else if(isSameTag("RigJoint")) {
			PfxRigJointInfo joint;
			if(readRigJoint(joint)) {
				addRigJoint(joint);
			}
		}
	}
}

void PfxCommonImport::readRigConvexMeshes()
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return;
			}
		}
		else if(isSameTag("RigConvexMesh")) {
			PfxRigConvexMeshInfo convexmesh;
			if(readRigConvexMesh(convexmesh)) {
				addRigConvexMesh(convexmesh);
			}
		}
	}
}

void PfxCommonImport::readRigLargeMeshes()
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return;
			}
		}
		else if(isSameTag("RigLargeMesh")) {
			PfxRigLargeMeshInfo largemesh;
			if(readRigLargeMesh(largemesh)) {
				//PRINTF("readRigLargeMesh verts %d indices %d\n",largemesh.verts.size()/3,largemesh.indices.size());
				addRigLargeMesh(largemesh);
			}
		}
	}
}

void PfxCommonImport::readRigNonContactPairs()
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return;
			}
		}
		else if(isSameTag("pair")) {
			PfxRigNonContactPairInfo pair;
			readPair(pair);
			addRigNonContactPair(pair);
		}
	}
}

PfxBool PfxCommonImport::readRigBody(PfxRigBodyInfo &body)
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return true;
			}
		}
		else if(isSameTag("Shape")) {
			PfxRigShapeInfo shape;
			if(readRigShape(shape)) {
				body.shapes.push_back(shape);
			}
		}
		READ_STRING("name",body.name)
		READ_FLOAT("mass",body.mass)
		READ_MATRIX3("inertia",body.inertia)
		READ_FLOAT("restitution",body.restitution)
		READ_FLOAT("friction",body.friction)
	}

	return false;
}

PfxBool PfxCommonImport::readRigShape(PfxRigShapeInfo &shape)
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return true;
			}
		}
		else if(isSameTag("shapeType")) {
			PfxUInt32 param;
			readUInt32(param);
			shape.shapeType = (PfxRigShapeType)param;
		}
		READ_UINT32("contactFilterSelf",shape.contactFilterSelf)
		READ_UINT32("contactFilterTarget",shape.contactFilterTarget)
		READ_QUAT("orientation",shape.relativeOrientation)
		READ_VEC3("position",shape.relativePosition)
		READ_FLOAT3("vfData",shape.vfData)
		READ_INDEX3("viData",shape.viData)
	}

	return false;
}

PfxBool PfxCommonImport::readRigState(PfxRigStateInfo &state)
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return true;
			}
		}
		else if(isSameTag("moveType")) {
			PfxUInt32 param;
			readUInt32(param);
			state.moveType = (PfxRigMoveType)param;
		}
		READ_STRING("name",state.name)
		READ_UINT16("contactFilterSelf",state.contactFilterSelf)
		READ_UINT16("contactFilterTarget",state.contactFilterTarget)
		READ_STRING("bodyName",state.bodyName)
		READ_BOOL("sleeping",state.sleeping)
		READ_BOOL("useSleep",state.useSleep)
		READ_BOOL("useCcd",state.useCcd)
		READ_BOOL("useContactCallback",state.useContactCallback)
		READ_BOOL("useSleepCallback",state.useSleepCallback)
		READ_VEC3("position",state.position)
		READ_QUAT("orientation",state.orientation)
		READ_VEC3("linearVelocity",state.linearVelocity)
		READ_VEC3("angularVelocity",state.angularVelocity)
		READ_FLOAT("linearDamping",state.linearDamping)
		READ_FLOAT("angularDamping",state.angularDamping)
	}

	return false;
}

PfxBool PfxCommonImport::readRigJoint(PfxRigJointInfo &joint)
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return true;
			}
		}
		else if(isSameTag("jointType")) {
			PfxUInt32 param;
			readUInt32(param);
			joint.jointType = (PfxRigJointType)param;
		}
		READ_STRING("name",joint.name)
		READ_STRING("parentName",joint.parentName)
		READ_STRING("childName",joint.childName)
		READ_VEC3("anchor",joint.anchor)
		READ_VEC3("axis",joint.axis)
		READ_FLOAT("lowerLimit1",joint.lowerLimit1)
		READ_FLOAT("upperLimit1",joint.upperLimit1)
		READ_FLOAT("lowerLimit2",joint.lowerLimit2)
		READ_FLOAT("upperLimit2",joint.upperLimit2)
		READ_FLOAT("distance",joint.distance)
		READ_FLOAT("linearDamping",joint.linearDamping)
		READ_FLOAT("angularDamping",joint.angularDamping)
		READ_FLOAT("linearImpulseWeight",joint.linearImpulseWeight)
		READ_FLOAT("angularImpulseWeight",joint.angularImpulseWeight)
		READ_FLOAT("linearBias",joint.linearBias)
		READ_FLOAT("angularBias",joint.angularBias)
		READ_FLOAT("maxLinearImpulse",joint.maxLinearImpulse)
		READ_FLOAT("maxAngularImpulse",joint.maxAngularImpulse)
		READ_FLOAT("breakableLimit",joint.breakableLimit)
	}

	return false;
}

PfxBool PfxCommonImport::readRigConvexMesh(PfxRigConvexMeshInfo &convexmesh)
{
	int indent = 0;
	PfxUInt32 numVerts = 0;
	PfxUInt32 numIndices = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				// check
				if(numVerts == convexmesh.verts.size()/3 && 
					numIndices == convexmesh.indices.size()) {
					return true;
				}
				else {
					return false;
				}
			}
		}
		else if(isSameTag("Verts")) {
			readVerts(convexmesh.verts);
		}
		else if(isSameTag("Indices")) {
			readIndices(convexmesh.indices);
		}
		READ_UINT32("id",convexmesh.id)
		READ_UINT32("numVerts",numVerts)
		READ_UINT32("numIndices",numIndices)
	}

	return false;
}

PfxBool PfxCommonImport::readRigLargeMesh(PfxRigLargeMeshInfo &largemesh)
{
	int indent = 0;
	PfxUInt32 numVerts = 0;
	PfxUInt32 numIndices = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				// check
				if(numVerts == largemesh.verts.size()/3 && 
					numIndices == largemesh.indices.size()) {
					return true;
				}
				else {
					return false;
				}
			}
		}
		else if(isSameTag("Verts")) {
			readVerts(largemesh.verts);
		}
		else if(isSameTag("Indices")) {
			readIndices(largemesh.indices);
		}
		READ_UINT32("id",largemesh.id)
		READ_UINT32("numFacetsLimit",largemesh.numFacetsLimit)
		READ_FLOAT("islandsRatio",largemesh.islandsRatio)
		READ_UINT32("numVerts",numVerts)
		READ_UINT32("numIndices",numIndices)
	}

	return false;
}

void PfxCommonImport::readVerts(vector<PfxFloat> &verts)
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return;
			}
		}
		else if(isSameTag("vtx")) {
			PfxFloat3 vtx;
			readFloat3(vtx);
			verts.push_back(vtx[0]);
			verts.push_back(vtx[1]);
			verts.push_back(vtx[2]);
		}
	}
}

void PfxCommonImport::readIndices(vector<PfxUInt16> &indices)
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return;
			}
		}
		else if(isSameTag("idx")) {
			PfxUInt32 idx[3];
			readIndex3(idx[0],idx[1],idx[2]);
			indices.push_back(idx[0]);
			indices.push_back(idx[1]);
			indices.push_back(idx[2]);
		}
	}
}

PfxBool PfxCommonImport::readPair(PfxRigNonContactPairInfo &pair)
{
	int indent = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return true;
			}
		}
		else if(isSameTag("rigA")) {
			char str[IMPORT_STRING_MAX];
			readString(str,IMPORT_STRING_MAX);
			pair.rigA = str;
		}
		else if(isSameTag("rigB")) {
			char str[IMPORT_STRING_MAX];
			readString(str,IMPORT_STRING_MAX);
			pair.rigB = str;
		}
	}

	return false;
}

void PfxCommonImport::readRigidBodies()
{
	while(getNextTag()) {
		if(isSameTag("RigWorld")) {
			PfxRigWorldInfo world;
			if(readRigWorld(world)) {
				mRigWorldInfo = world;
			}
		}
		else if(isSameTag("RigNonContactPairArray")) {
			readRigNonContactPairs();
		}
		else if(isSameTag("RigBodyArray")) {
			readRigBodies();
		}
		else if(isSameTag("RigStateArray")) {
			readRigStates();
		}
		else if(isSameTag("RigJointArray")) {
			readRigJoints();
		}
		else if(isSameTag("RigConvexMeshArray")) {
			readRigConvexMeshes();
		}
		else if(isSameTag("RigLargeMeshArray")) {
			readRigLargeMeshes();
		}
	}
}

PfxBool PfxCommonImport::doImport(const char *filename)
{
	if(!open(filename))
		return false;

	clear();

	while(getNextTag()) {
		if(isSameTag("Vender")) {
			char str[IMPORT_STRING_MAX];
			readString(str,IMPORT_STRING_MAX);
			//PRINTF("Vender %s\n",str);
		}
		else if(isSameTag("Version")) {
			char str[IMPORT_STRING_MAX];
			readString(str,IMPORT_STRING_MAX);
			//PRINTF("Version %s\n",str);
		}
		else if(isSameTag("RigidBodies")) {
			readRigidBodies();
		}
	}

	close();
	
	return true;
}

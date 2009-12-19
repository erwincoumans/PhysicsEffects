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

#include "PfxCommonExport.h"

void PfxCommonExport::writePreHeader()
{
	writeOpenTag("PhysicsEffectsFormat");
	writeString("Vender",PFX_FORMAT_VENDER);
	writeString("Version",PFX_FORMAT_VERSION);
	writeOpenTag("RigidBodies");
}

void PfxCommonExport::writePostHeader()
{
	writeCloseTag(); // RigidBodies
	writeCloseTag(); // PhysicsEffectsFormat
}

void PfxCommonExport::writeRigWorld()
{
	writeOpenTag("RigWorld");

	writeUInt32("maxInstances",mRigWorldInfo.maxInstances);
	writeUInt32("maxDynBodies",mRigWorldInfo.maxDynBodies);
	writeUInt32("maxPrimitives",mRigWorldInfo.maxPrimitives);
	writeUInt32("maxJoints",mRigWorldInfo.maxJoints);
	writeUInt32("maxSprings",mRigWorldInfo.maxSprings);
	writeUInt32("maxContactPairs",mRigWorldInfo.maxContactPairs);

	writeFloat3("worldExtent",mRigWorldInfo.worldExtent);
	writeFloat3("worldCenter",mRigWorldInfo.worldCenter);
	writeFloat3("gravity",mRigWorldInfo.gravity);
	
	writeUInt32("subStepCount",mRigWorldInfo.subStepCount);
	writeUInt32("contactIteration",mRigWorldInfo.contactIteration);
	writeUInt32("jointIteration",mRigWorldInfo.jointIteration);
	writeFloat("maxLinearVelocity",mRigWorldInfo.maxLinearVelocity);
	writeFloat("maxAngularVelocity",mRigWorldInfo.maxAngularVelocity);
	writeFloat("separateBias",mRigWorldInfo.separateBias);

	writeBool("sleepEnable",mRigWorldInfo.sleepEnable);
	writeUInt32("sleepInterval",mRigWorldInfo.sleepInterval);
	writeUInt32("sleepCount",mRigWorldInfo.sleepCount);
	writeFloat("sleepLinearVelocity",mRigWorldInfo.sleepLinearVelocity);
	writeFloat("sleepAngularVelocity",mRigWorldInfo.sleepAngularVelocity);
	writeFloat("wakeLinearVelocity",mRigWorldInfo.wakeLinearVelocity);
	writeFloat("wakeAngularVelocity",mRigWorldInfo.wakeAngularVelocity);

	writeBool("ccdEnable",mRigWorldInfo.ccdEnable);
	writeBool("deformMeshEnable",mRigWorldInfo.deformMeshEnable);
	
	writeCloseTag();
}

void PfxCommonExport::writeRigBodies()
{
	writeOpenTag("RigBodyArray");
	writeUInt32("count",mRigBodies.size());
	for(PfxUInt32 i=0;i<mRigBodies.size();i++) {
		writeRigBody(mRigBodies[i]);
	}
	writeCloseTag();
}

void PfxCommonExport::writeRigStates()
{
	writeOpenTag("RigStateArray");
	writeUInt32("count",mRigStates.size());
	for(PfxUInt32 i=0;i<mRigStates.size();i++) {
		writeRigState(mRigStates[i]);
	}
	writeCloseTag();
}

void PfxCommonExport::writeRigJoints()
{
	writeOpenTag("RigJointArray");
	writeUInt32("count",mRigJoints.size());
	for(PfxUInt32 i=0;i<mRigJoints.size();i++) {
		writeRigJoint(mRigJoints[i]);
	}
	writeCloseTag();
}

void PfxCommonExport::writeRigConvexMeshes()
{
	writeOpenTag("RigConvexMeshArray");
	writeUInt32("count",mRigConvexMeshes.size());
	for(PfxUInt32 i=0;i<mRigConvexMeshes.size();i++) {
		writeRigConvexMesh(mRigConvexMeshes[i]);
	}
	writeCloseTag();
}

void PfxCommonExport::writeRigLargeMeshes()
{
	writeOpenTag("RigLargeMeshArray");
	writeUInt32("count",mRigLargeMeshes.size());
	for(PfxUInt32 i=0;i<mRigLargeMeshes.size();i++) {
		writeRigLargeMesh(mRigLargeMeshes[i]);
	}
	writeCloseTag();
}

void PfxCommonExport::writeRigNonContactPairs()
{
	writeOpenTag("RigNonContactPairArray");
	writeUInt32("count",mRigNonContactPairs.size());
	for(PfxUInt32 i=0;i<mRigNonContactPairs.size();i++) {
		writeOpenTag("pair");
		writeString("rigA",mRigNonContactPairs[i].rigA.c_str());
		writeString("rigB",mRigNonContactPairs[i].rigB.c_str());
		writeCloseTag();
	}
	writeCloseTag();
}

void PfxCommonExport::writeRigBody(const PfxRigBodyInfo &info)
{
	writeOpenTag("RigBody");
	writeString("name",info.name.c_str());
	writeFloat("mass",info.mass);
	writeMatrix3("inertia",info.inertia);
	writeFloat("restitution",info.restitution);
	writeFloat("friction",info.friction);
	writeUInt32("numShapes",info.shapes.size());
	for(PfxUInt32 i=0;i<info.shapes.size();i++) {
		const PfxRigShapeInfo &shape = info.shapes[i];

		writeOpenTag("Shape");
		writeUInt32("shapeType",shape.shapeType);
		writeUInt32("contactFilterSelf",shape.contactFilterSelf);
		writeUInt32("contactFilterTarget",shape.contactFilterTarget);
		writeFloat4("orientation",shape.relativeOrientation);
		writeFloat3("position",shape.relativePosition);
		switch(shape.shapeType) {
			case RigShapeTypeSphere:
			case RigShapeTypeBox:
			case RigShapeTypeCapsule:
			writeFloat3("vfData",shape.vfData[0],shape.vfData[1],shape.vfData[2]);
			break;
			
			case RigShapeTypeConvexMesh:
			case RigShapeTypeLargeMesh:
			writeIndex3("viData",shape.viData[0],shape.viData[1],shape.viData[2]);
			break;

			default:
			break;
		}
		writeCloseTag();
	}
	writeCloseTag();
}

void PfxCommonExport::writeRigState(const PfxRigStateInfo &info)
{
	writeOpenTag("RigState");
	
	writeString("name",info.name.c_str());
	writeUInt32("moveType",info.moveType);
	writeUInt32("contactFilterSelf",info.contactFilterSelf);
	writeUInt32("contactFilterTarget",info.contactFilterTarget);
	writeString("bodyName",info.bodyName.c_str());
	writeBool("sleeping",info.sleeping);
	writeBool("useSleep",info.useSleep);
	writeBool("useCcd",info.useCcd);
	writeBool("useContactCallback",info.useContactCallback);
	writeBool("useSleepCallback",info.useSleepCallback);
	
	writeFloat3("position",info.position);
	writeFloat4("orientation",info.orientation);
	writeFloat3("linearVelocity",info.linearVelocity);
	writeFloat3("angularVelocity",info.angularVelocity);
	writeFloat("linearDamping",info.linearDamping);
	writeFloat("angularDamping",info.angularDamping);

	writeCloseTag();
}

void PfxCommonExport::writeRigJoint(const PfxRigJointInfo &info)
{
	writeOpenTag("RigJoint");
	
	writeString("name",info.name.c_str());
	writeUInt32("jointType",info.jointType);
	writeString("parentName",info.parentName.c_str());
	writeString("childName",info.childName.c_str());
	writeFloat3("anchor",info.anchor);
	writeFloat3("axis",info.axis);
	
	writeFloat("lowerLimit1",info.lowerLimit1);
	writeFloat("upperLimit1",info.upperLimit1);
	writeFloat("lowerLimit2",info.lowerLimit2);
	writeFloat("upperLimit2",info.upperLimit2);
	writeFloat("distance",info.distance);
	
	writeFloat("linearDamping",info.linearDamping);
	writeFloat("angularDamping",info.angularDamping);
	writeFloat("linearImpulseWeight",info.linearImpulseWeight);
	writeFloat("angularImpulseWeight",info.angularImpulseWeight);
	writeFloat("linearBias",info.linearBias);
	writeFloat("angularBias",info.angularBias);
	writeFloat("maxLinearImpulse",info.maxLinearImpulse);
	writeFloat("maxAngularImpulse",info.maxAngularImpulse);
	writeFloat("breakableLimit",info.breakableLimit);
	
	writeCloseTag();
}

void PfxCommonExport::writeRigConvexMesh(const PfxRigConvexMeshInfo &info)
{
	writeOpenTag("RigConvexMesh");

	writeUInt32("id",info.id);
	writeUInt32("numVerts",(PfxUInt32)info.verts.size()/3);
	writeUInt32("numIndices",(PfxUInt32)info.indices.size());

	writeOpenTag("Verts");
	for(PfxUInt32 i=0;i<info.verts.size()/3;i++) {
		writeFloat3("vtx",PfxFloat3(info.verts[i*3],info.verts[i*3+1],info.verts[i*3+2]));
	}
	writeCloseTag();
	writeOpenTag("Indices");
	for(PfxUInt32 i=0;i<info.indices.size();i+=3) {
		writeIndex3("idx",info.indices[i],info.indices[i+1],info.indices[i+2]);
	}
	writeCloseTag();

	writeCloseTag();
}

void PfxCommonExport::writeRigLargeMesh(const PfxRigLargeMeshInfo &info)
{
	writeOpenTag("RigLargeMesh");

	writeUInt32("id",info.id);
	writeUInt32("numFacetsLimit",(PfxUInt32)info.numFacetsLimit);
	writeFloat("islandsRatio",info.islandsRatio);

	writeUInt32("numVerts",(PfxUInt32)info.verts.size()/3);
	writeUInt32("numIndices",(PfxUInt32)info.indices.size());

	writeOpenTag("Verts");
	for(PfxUInt32 i=0;i<info.verts.size()/3;i++) {
		writeFloat3("vtx",PfxFloat3(info.verts[i*3],info.verts[i*3+1],info.verts[i*3+2]));
	}
	writeCloseTag();
	writeOpenTag("Indices");
	for(PfxUInt32 i=0;i<info.indices.size();i+=3) {
		writeIndex3("idx",info.indices[i],info.indices[i+1],info.indices[i+2]);
	}
	writeCloseTag();

	writeCloseTag();
}

PfxBool PfxCommonExport::doExport(const char *filename)
{
	if(!open(filename))
		return false;

	// PreHeader
	writePreHeader();
	
	// World
	writeRigWorld();
	
	// NonContactPairs
	writeRigNonContactPairs();
	
	// Bodies
	writeRigBodies();
	
	// States
	writeRigStates();
	
	// Joints
	writeRigJoints();
	
	// ConvexMeshes
	writeRigConvexMeshes();

	// LargeTriMeshes
	writeRigLargeMeshes();

	// PostHeader
	writePostHeader();
	
	close();
	
	return true;
}

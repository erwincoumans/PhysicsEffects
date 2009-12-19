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

#include "RigidBodies.h"

#define PFX_FMT_VER "PfxSnapshot v0.7"

///////////////////////////////////////////////////////////////////////////////
// 剛体スナップショット

bool RigidBodies::saveSnapshot(const char *filename)
{
	if(!mWriter.open(filename))
		return false;

	// PreHeader
	writePreHeader();
	
	// World
	writeWorld();
	
	// NonContactPair
	writeNonContactPair();
	
	// Bodies
	writeBodies();
	
	// States
	writeStates();
	
	// Joints
	writeJoints();
	
	// Mesh
	writeMeshes();

	// PostHeader
	writePostHeader();
	
	mWriter.close();
	
	return true;
}

///////////////////////////////////////////////////////////////////////////////
// ヘッダ

void RigidBodies::writePreHeader()
{
	mWriter.writeOpenTag("PhysicsEffectsSDK");
	mWriter.writeString("FormatVersion",PFX_FMT_VER);
	mWriter.writeOpenTag("RigidBodies");
}

void RigidBodies::writePostHeader()
{
	mWriter.writeCloseTag();
	mWriter.writeCloseTag();
}

///////////////////////////////////////////////////////////////////////////////
// ワールド

void RigidBodies::writeWorld()
{
	mWriter.writeOpenTag("WorldProperty");

	mWriter.writeUint32("maxInstances",worldProperty.maxInstances);
	mWriter.writeUint32("maxDynBodies",worldProperty.maxDynBodies);
	mWriter.writeUint32("maxPrimitives",worldProperty.maxPrimitives);
	mWriter.writeUint32("maxJoints",worldProperty.maxJoints);
	mWriter.writeUint32("maxSprings",worldProperty.maxSprings);
	mWriter.writeUint32("maxContactPairs",worldProperty.maxContactPairs);

	mWriter.writeVector3("worldCenter",worldProperty.worldCenter);
	mWriter.writeVector3("worldExtent",worldProperty.worldExtent);
	mWriter.writeVector3("gravity",worldProperty.gravity);
	
	mWriter.writeUint32("contactIteration",worldProperty.contactIteration);
	mWriter.writeUint32("jointIteration",worldProperty.jointIteration);
	mWriter.writeFloat("maxLinearVelocity",worldProperty.maxLinearVelocity);
	mWriter.writeFloat("maxAngularVelocity",worldProperty.maxAngularVelocity);
	mWriter.writeFloat("separateBias",worldProperty.separateBias);

	mWriter.writeBool("sleepEnable",worldProperty.sleepEnable);
	mWriter.writeUint32("sleepInterval",worldProperty.sleepInterval);
	mWriter.writeUint32("sleepCount",worldProperty.sleepCount);
	mWriter.writeFloat("sleepLinearVelocity",worldProperty.sleepLinearVelocity);
	mWriter.writeFloat("sleepAngularVelocity",worldProperty.sleepAngularVelocity);
	mWriter.writeFloat("wakeLinearVelocity",worldProperty.wakeLinearVelocity);
	mWriter.writeFloat("wakeAngularVelocity",worldProperty.wakeAngularVelocity);

	mWriter.writeBool("ccdEnable",worldProperty.ccdEnable);
	mWriter.writeBool("deformMeshEnable",worldProperty.deformMeshEnable);
	
	mWriter.writeCloseTag();
}

///////////////////////////////////////////////////////////////////////////////
// 剛体

void RigidBodies::writeBodies()
{
	mWriter.writeOpenTag("TrbDynBodyArray");
	for(uint32_t i=0;i<numBodies;i++) {
		writeTrbDynBody(bodies[i],i);
	}
	mWriter.writeCloseTag();
}

void RigidBodies::writeTrbDynBody(const TrbDynBody &body,uint32_t id)
{
	CollObject &coll = *body.getCollObject();

	mWriter.writeOpenTag("TrbDynBody");
	mWriter.writeUint32("id",id);
	mWriter.writeFloat("mass",body.getMass());
	mWriter.writeMatrix3("inertia",body.getBodyInertia());
	mWriter.writeFloat("restitution",body.getElasticity());
	mWriter.writeFloat("friction",body.getFriction());
	mWriter.writeUint32("numPrims",coll.getNumPrims());

	PrimIterator itrPrim(coll);
	for(uint32_t i=0;i<coll.getNumPrims();i++,++itrPrim) {
		const CollPrim &prim = *itrPrim;

		mWriter.writeOpenTag("Prim");
		mWriter.writeQuat("orientation",prim.getObjectRelOrientation());
		mWriter.writeVector3("position",prim.getObjectRelPosition());
		mWriter.writeUint32("type",prim.getType());
		switch(prim.getType()) {
			case SPHERE:case BOX:case CAPSULE:
			mWriter.writeFloat3("vfData",prim.getPrimDataFloat(0),prim.getPrimDataFloat(1),prim.getPrimDataFloat(2));
			break;
			
			case HEIGHTFIELD:case CONVEXMESH:case LARGEMESH:
			mWriter.writeIndex3("viData",prim.getPrimDataInteger(0),prim.getPrimDataInteger(1),prim.getPrimDataInteger(2));
			break;

			default:
			break;
		}
		mWriter.writeCloseTag();
	}
	mWriter.writeCloseTag();
}

///////////////////////////////////////////////////////////////////////////////
// 剛体ステート

void RigidBodies::writeStates()
{
	mWriter.writeOpenTag("TrbStateArray");
	for(uint32_t i=0;i<numInstances;i++) {
		writeTrbState(statesBuffer[readBuffer][i],i);
	}
	mWriter.writeCloseTag();
}

void RigidBodies::writeTrbState(const TrbState &state,uint32_t id)
{
	mWriter.writeOpenTag("TrbState");
	
	mWriter.writeUint32("id",id);
	mWriter.writeUint32("moveType",state.getMoveType());
	mWriter.writeUint32("contactFilterSelf",state.getContactFilterSelf());
	mWriter.writeUint32("contactFilterTarget",state.getContactFilterTarget());
	mWriter.writeUint32("trbBodyIdx",state.trbBodyIdx);
	mWriter.writeBool("sleeping",state.isAsleep());
	mWriter.writeUint32("useSleep",state.getUseSleep());
	mWriter.writeUint32("useCcd",state.getUseCcd());
	mWriter.writeUint32("useContactCallback",state.getUseContactCallback());
	mWriter.writeUint32("useSleepCallback",state.getUseSleepCallback());
	mWriter.writeUint32("sleepCount",state.getSleepCount());
	
	Vector3 p(state.getPosition());
	Quat	q(state.getOrientation());
	Vector3 v(state.getLinearVelocity());
	Vector3 w(state.getAngularVelocity());

	mWriter.writeVector3("position",p);
	mWriter.writeQuat("orientation",q);
	mWriter.writeVector3("linearVelocity",v);
	mWriter.writeVector3("angularVelocity",w);
	mWriter.writeFloat("linearDamping",state.linearDamping);
	mWriter.writeFloat("angularDamping",state.angularDamping);
	
	mWriter.writeCloseTag();
}

///////////////////////////////////////////////////////////////////////////////
// ジョイント

void RigidBodies::writeJoints()
{
	mWriter.writeOpenTag("JointArray");
	for(uint32_t i=0;i<numJoints;i++) {
		writeJoint(joints[i]);
	}
	mWriter.writeCloseTag();
}

void RigidBodies::writeJoint(const Joint &joint)
{
	mWriter.writeOpenTag("Joint");

	mWriter.writeUint32("jointFlag",joint.jointFlag);
	mWriter.writeUint32("jointType",joint.jointType);
	mWriter.writeUint32("stateIndexA",joint.stateIndexA);
	mWriter.writeUint32("stateIndexB",joint.stateIndexB);

	mWriter.writeFloat3("linLowerLimit",joint.lowerLimit[0],joint.lowerLimit[1],joint.lowerLimit[2]);
	mWriter.writeFloat3("angLowerLimit",joint.lowerLimit[3],joint.lowerLimit[4],joint.lowerLimit[5]);
	mWriter.writeFloat3("linUpperLimit",joint.upperLimit[0],joint.upperLimit[1],joint.upperLimit[2]);
	mWriter.writeFloat3("angUpperLimit",joint.upperLimit[3],joint.upperLimit[4],joint.upperLimit[5]);

	mWriter.writeFloat("linearDamping",joint.linearDamping);
	mWriter.writeFloat("angularDamping",joint.angularDamping);

	mWriter.writeFloat("linearBias",joint.linearBias);
	mWriter.writeFloat("angularBias",joint.angularBias);

	mWriter.writeFloat("maxLinearImpulse",joint.maxLinearImpulse);
	mWriter.writeFloat("maxAngularImpulse",joint.maxAngularImpulse);

	mWriter.writeFloat("linearImpulseWeight",joint.linearImpulseWeight);
	mWriter.writeFloat("angularImpulseWeight",joint.angularImpulseWeight);

	mWriter.writeFloat("breakableLimit",joint.breakableLimit);

	mWriter.writeVector3("anchorA",joint.anchorA);
	mWriter.writeVector3("anchorB",joint.anchorB);
	mWriter.writeMatrix3("frameA",joint.frameA);
	mWriter.writeMatrix3("frameB",joint.frameB);
	mWriter.writeMatrix3("targetFrame",joint.targetFrame);

	mWriter.writeCloseTag();
}

///////////////////////////////////////////////////////////////////////////////
// メッシュ

void RigidBodies::writeMeshes()
{
	// 最初にメッシュを探索
	const int maxMesh = 255;
	int numConvexMesh = 0;
	int numLargeMesh = 0;
	uint32_t *convexMeshAddr = (uint32_t*)mPool->allocate(sizeof(uint32_t)*maxMesh);
	uint32_t *largeMeshAddr = (uint32_t*)mPool->allocate(sizeof(uint32_t)*maxMesh);
	
	for(uint32_t i=0;i<numBodies;i++) {
		const CollObject &coll = *bodies[i].getCollObject();
		PrimIterator itrPrim(coll);
		for(uint32_t j=0;j<coll.getNumPrims();j++,++itrPrim) {
			const CollPrim &prim = *itrPrim;

			if(prim.getType() == CONVEXMESH) {
				uint32_t meshAddr = (uint32_t)prim.getConvexMesh();
				bool findSame = false;
				for(int k=0;k<numConvexMesh;k++) {
					if(convexMeshAddr[k] == meshAddr) {
						findSame = true;
						break;
					}
				}
				if(!findSame) {
					convexMeshAddr[numConvexMesh++] = meshAddr;
				}
			}
			else if(prim.getType() == LARGEMESH) {
				uint32_t meshAddr = (uint32_t)prim.getLargeMesh();
				bool findSame = false;
				for(int k=0;k<numLargeMesh;k++) {
					if(largeMeshAddr[k] == meshAddr) {
						findSame = true;
						break;
					}
				}
				if(!findSame) {
					largeMeshAddr[numLargeMesh++] = meshAddr;
				}
			}
		}
	}
	
	// メッシュを書き出す
	mWriter.writeOpenTag("ConvexMeshArray");
	for(int k=0;k<numConvexMesh;k++) {
		writeConvexMesh(*((ConvexMesh*)convexMeshAddr[k]));
	}
	mWriter.writeCloseTag();

	// ラージメッシュを書き出す
	mWriter.writeOpenTag("LargeMeshArray");
	for(int k=0;k<numLargeMesh;k++) {
		writeLargeMesh(*((LargeTriMesh*)largeMeshAddr[k]));
	}
	mWriter.writeCloseTag();

	mPool->deallocate(largeMeshAddr);
	mPool->deallocate(convexMeshAddr);
}

void RigidBodies::writeConvexMesh(const ConvexMesh &mesh)
{
	mWriter.writeOpenTag("ConvexMesh");
	
	mWriter.writeUint32("ID",(uint32_t)&mesh);
	mWriter.writeUint32("numVerts",(uint32_t)mesh.numVerts);
	mWriter.writeUint32("numIndices",(uint32_t)mesh.numIndices);
		mWriter.writeOpenTag("Verts");
		for(uint32_t i=0;i<mesh.numVerts;i++) {
			mWriter.writeVector3("vtx",mesh.verts[i]);
		}
		mWriter.writeCloseTag();
		mWriter.writeOpenTag("Indices");
		for(uint32_t i=0;i<mesh.numIndices;i+=3) {
			mWriter.writeIndex3("idx",mesh.indices[i],mesh.indices[i+1],mesh.indices[i+2]);
		}
		mWriter.writeCloseTag();
	mWriter.writeCloseTag();
}

void RigidBodies::writeLargeMesh(const LargeTriMesh &largeMesh)
{
	mWriter.writeOpenTag("LargeTriMesh");
	mWriter.writeUint32("ID",(uint32_t)&largeMesh);
	mWriter.writeUint32("numIslands",(uint32_t)largeMesh.numIslands);
	mWriter.writeUint32("maxIslands",(uint32_t)largeMesh.maxIslands);
	for(uint32_t l=0;l<largeMesh.numIslands;l++) {
		TriMesh &mesh = largeMesh.islands[l];

		mWriter.writeOpenTag("TriMesh");
			mWriter.writeUint32("numVerts",(uint32_t)mesh.numVerts);
			mWriter.writeUint32("numIndices",(uint32_t)mesh.numFacets*3);
			mWriter.writeOpenTag("Verts");
			for(uint32_t i=0;i<mesh.numVerts;i++) {
				mWriter.writeVector3("vtx",mesh.verts[i]);
			}
			mWriter.writeCloseTag();
			mWriter.writeOpenTag("Indices");
			for(uint32_t i=0;i<mesh.numFacets;i++) {
				MeshFacet &facet = mesh.facets[i];
				mWriter.writeIndex3("idx",facet.vertIndices[0],facet.vertIndices[1],facet.vertIndices[2]);
			}
			mWriter.writeCloseTag();
		mWriter.writeCloseTag();
	}
	mWriter.writeCloseTag();
}

///////////////////////////////////////////////////////////////////////////////
// Non Contact Pair

void RigidBodies::writeNonContactPair()
{
	mWriter.writeOpenTag("NonContactPair");

	for(uint32_t i=0;i<numInstances;i++) {
		for(uint32_t j=i+1;j<numInstances;j++) {
			if(!isCollidablePair(i,j)) {
				mWriter.writeIndex3("pair",i,j,0);
			}
		}
	}
	
	mWriter.writeCloseTag();
}

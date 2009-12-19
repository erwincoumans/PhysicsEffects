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

#include "PfxSnapshotReader.h"
#include "Util/MeshUtil.h"

#define IMPORT_STRING_MAX 256

#define READ_STRING(tag,param)	else if(isSameTag(tag)) {char str[IMPORT_STRING_MAX];readString(str,IMPORT_STRING_MAX);param=str;}
#define READ_BOOL(tag,param)	else if(isSameTag(tag)) {readBool(param);}
#define READ_UINT32(tag,param)	else if(isSameTag(tag)) {readUint32(param);}
#define READ_UINT16(tag,param)	else if(isSameTag(tag)) {uint32_t tmp;readUint32(tmp);param=(uint16_t)tmp;}
#define READ_UINT8(tag,param)	else if(isSameTag(tag)) {uint32_t tmp;readUint32(tmp);param=(uint8_t)tmp;}
#define READ_FLOAT(tag,param)	else if(isSameTag(tag)) {readFloat(param);}
#define READ_VEC3(tag,param)	else if(isSameTag(tag)) {readVector3(param);}
#define READ_QUAT(tag,param)	else if(isSameTag(tag)) {readQuat(param);}
#define READ_MATRIX3(tag,param)	else if(isSameTag(tag)) {readMatrix3(param);}
#define READ_FLOAT3(tag,param)	else if(isSameTag(tag)) {readFloat3(*(param),*((param)+1),*((param)+2));}
#define READ_INDEX3(tag,param)	else if(isSameTag(tag)) {readIndex3(*(param),*((param)+1),*((param)+2));}

bool PfxSnapshotReader::readWorldProperty(WorldProperty &wp)
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
		READ_UINT32("maxInstances",wp.maxInstances)
		READ_UINT32("maxDynBodies",wp.maxDynBodies)
		READ_UINT32("maxPrimitives",wp.maxPrimitives)
		READ_UINT32("maxJoints",wp.maxJoints)
		READ_UINT32("maxSprings",wp.maxSprings)
		READ_UINT32("maxContactPairs",wp.maxContactPairs)
		READ_VEC3("worldCenter",wp.worldCenter)
		READ_VEC3("worldExtent",wp.worldExtent)
		READ_VEC3("gravity",wp.gravity)
		READ_UINT8("subStepCount",wp.subStepCount)
		READ_UINT8("contactIteration",wp.contactIteration)
		READ_UINT8("jointIteration",wp.jointIteration)
		READ_FLOAT("maxLinearVelocity",wp.maxLinearVelocity)
		READ_FLOAT("maxAngularVelocity",wp.maxAngularVelocity)
		READ_FLOAT("separateBias",wp.separateBias)
		READ_BOOL("sleepEnable",wp.sleepEnable)
		READ_UINT16("sleepInterval",wp.sleepInterval)
		READ_UINT16("sleepCount",wp.sleepCount)
		READ_FLOAT("sleepLinearVelocity",wp.sleepLinearVelocity)
		READ_FLOAT("sleepAngularVelocity",wp.sleepAngularVelocity)
		READ_FLOAT("wakeLinearVelocity",wp.wakeLinearVelocity)
		READ_FLOAT("wakeAngularVelocity",wp.wakeAngularVelocity)
		READ_BOOL("ccdEnable",wp.ccdEnable)
		READ_BOOL("deformMeshEnable",wp.deformMeshEnable)
	}
	
	return false;
}

void PfxSnapshotReader::readTrbDynBodyArray()
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
		else if(isSameTag("TrbDynBody")) {
			readTrbDynBody();
		}
	}
}

void PfxSnapshotReader::readTrbStateArray()
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
		else if(isSameTag("TrbState")) {
			readTrbState();
		}
	}
}

void PfxSnapshotReader::readJointArray()
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
		else if(isSameTag("Joint")) {
			readJoint();
		}
	}
}

void PfxSnapshotReader::readConvexMeshArray()
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
		else if(isSameTag("ConvexMesh")) {
			ConvexMesh mesh;
			uint32_t id = readConvexMesh(mesh);
			if(id > 0 && mesh.numIndices > 0 && mesh.numVerts > 0) {
				mMeshArray->insert(pair<uint32_t,ConvexMesh>(id,mesh));
			}
		}
	}
}

void PfxSnapshotReader::readLargeMeshArray()
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
		else if(isSameTag("LargeTriMesh")) {
			LargeTriMesh lmesh;
			uint32_t id = readLargeMesh(lmesh);
			MeshUtil::updateLargeTriMesh(lmesh);
			if(id > 0 && lmesh.numIslands > 0) {
				PRINTF("read large mesh numIslands %03d\n",lmesh.numIslands);
				for(uint32_t i=0;i<lmesh.numIslands;i++) {
					PRINTF("    island %03d numVerts %d numFacets %d\n",i,lmesh.islands[i].numVerts,lmesh.islands[i].numFacets);
				}
				mLargeMeshArray->insert(pair<uint32_t,LargeTriMesh>(id,lmesh));
			}
		}
	}
}

void PfxSnapshotReader::readNonContactPair()
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
		else if(isSameTag("ncp")) {
			uint32_t data[3];
			readIndex3(data[0],data[1],data[2]);
			mRigidBodies->setNonContactPair(data[0],data[1]);
		}
		else if(isSameTag("pair")) {
			uint32_t data[3];
			readIndex3(data[0],data[1],data[2]);
			mRigidBodies->appendNonContactPair(data[0],data[1]);
		}
	}
}

bool PfxSnapshotReader::readTrbDynBody()
{
	int indent = 0;
	uint32_t numPrims = 0;
	float mass = 0.0f;
	float elasticity = 0.0f;
	float friction = 0.0f;
	Matrix3 inertia = Matrix3::identity();
	CollObject *coll = NULL;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				if(coll) {
					coll->finish();

					RigidBodyProperty bodyProperty;
					bodyProperty.collObject = coll;
					TrbDynBody &body = *mRigidBodies->createRigidBody(bodyProperty);

					body.setMass(mass);
					body.setElasticity(elasticity);
					body.setFriction(friction);
					body.setBodyInertia(inertia);
				}
				return true;
			}
		}
		else if(isSameTag("mass")) {
			readFloat(mass);
		}
		else if(isSameTag("inertia")) {
			readMatrix3(inertia);
		}
		else if(isSameTag("restitution")) {
			readFloat(elasticity);
		}
		else if(isSameTag("friction")) {
			readFloat(friction);
		}
		else if(isSameTag("numPrims")) {
			readUint32(numPrims);
			if(numPrims > 0) {
				coll = mRigidBodies->createCollObject(numPrims);
			}
		}
		else if(isSameTag("Prim") && coll) {
			readPrim(*coll);
		}
	}

	return false;
}

bool PfxSnapshotReader::readPrim(CollObject &coll)
{
	int indent = 0;

	Quat ori = Quat::identity();
	Vector3 pos(0.0f);
	PrimType primType = SPHERE;
	union {
		float    vfData[3];
		uint32_t viData[3];
	};
	
	viData[0] = viData[1] = viData[2] = 0;
	
	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				break;
			}
		}
		else if(isSameTag("orientation")) {
			readQuat(ori);
		}
		else if(isSameTag("position")) {
			readVector3(pos);
		}
		else if(isSameTag("type")) {
			uint32_t type;
			readUint32(type);
			primType = (PrimType)type;
		}
		else if(isSameTag("viData")) {
			readIndex3(viData[0],viData[1],viData[2]);
		}
		else if(isSameTag("vfData")) {
			readFloat3(vfData[0],vfData[1],vfData[2]);
		}
	}

	Transform3 trans(ori,pos);

	switch(primType) {
		case SPHERE:
		coll.addSphere(Sphere(vfData[0]),trans);
		break;

		case BOX:
		coll.addBox(Box(vfData[0],vfData[1],vfData[2]),trans);
		break;
 
		case CAPSULE:
		coll.addCapsule(Capsule(vfData[0],vfData[1]),trans);
		break;

		case CONVEXMESH:
		{
			map<uint32_t,ConvexMesh>::iterator i = mMeshArray->find(viData[0]);
			if(i != mMeshArray->end()) {
				coll.addConvex(&i->second,trans);
			}
		}
		break;

		case LARGEMESH:
		{
			map<uint32_t,LargeTriMesh>::iterator i = mLargeMeshArray->find(viData[0]);
			if(i != mLargeMeshArray->end()) {
				coll.setLargeMesh(&i->second);
			}
		}
		break;

		default:
		break;
	};

	return true;
}

bool PfxSnapshotReader::readTrbState()
{
	int indent = 0;

	uint32_t moveType = 0;
	uint32_t sleeping = 0;
	uint32_t useSleep = 1;
	uint32_t useCcd = 0;
	uint32_t useContactCallback = 0;
	uint32_t useSleepCallback = 0;
	uint32_t contactFilter = 0xffffffff;
	uint32_t contactFilterSelf = 0xffffffff;
	uint32_t contactFilterTarget = 0xffffffff;
	uint32_t trbBodyIdx = 0;
	uint32_t sleepCount = 0;
	TrbDynBody *trbBody = 0;
	Vector3  fX(0.0f);
	Quat     fQ = Quat::identity();
	Vector3  fV(0.0f);
	Vector3  fOmega(0.0f);
	float	 linearDamping = 1.0f;
	float    angularDamping = 0.99f;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				break;
			}
		}
		else if(isSameTag("moveType")) {
			readUint32(moveType);
		}
		else if(isSameTag("contactFilter")) {
			readUint32(contactFilter);
			contactFilterSelf = contactFilterTarget = contactFilter;
		}
		else if(isSameTag("contactFilterSelf")) {
			readUint32(contactFilterSelf);
		}
		else if(isSameTag("contactFilterTarget")) {
			readUint32(contactFilterTarget);
		}
		else if(isSameTag("trbBodyIdx")) {
			readUint32(trbBodyIdx);
			trbBody = mRigidBodies->getTrbDynBodyByIndex(trbBodyIdx);
		}
		else if(isSameTag("sleeping")) {
			readUint32(sleeping);
		}
		else if(isSameTag("useSleep")) {
			readUint32(useSleep);
		}
		else if(isSameTag("useCcd")) {
			readUint32(useCcd);
		}
		else if(isSameTag("useContactCallback")) {
			readUint32(useContactCallback);
		}
		else if(isSameTag("useSleepCallback")) {
			readUint32(useSleepCallback);
		}
		else if(isSameTag("sleepCount")) {
			readUint32(sleepCount);
		}
		else if(isSameTag("position")) {
			readVector3(fX);
		}
		else if(isSameTag("orientation")) {
			readQuat(fQ);
		}
		else if(isSameTag("linearVelocity")) {
			readVector3(fV);
		}
		else if(isSameTag("angularVelocity")) {
			readVector3(fOmega);
		}
		else if(isSameTag("linearDamping")) {
			readFloat(linearDamping);
		}
		else if(isSameTag("angularDamping")) {
			readFloat(angularDamping);
		}
	}
	
	InstanceProperty instProperty;
	instProperty.moveType = moveType;
	instProperty.contactFilterSelf = contactFilterSelf;
	instProperty.contactFilterTarget = contactFilterTarget;
	instProperty.position = fX;
	instProperty.orientation = fQ;
	instProperty.velocity = fV;
	instProperty.angularVelocity = fOmega;
	instProperty.linearDamping = linearDamping;
	instProperty.angularDamping = angularDamping;
	instProperty.rigidBody = trbBody;

	int instance = mRigidBodies->createInstance(instProperty);
	assert(instance >= 0);

	TrbState *state = mRigidBodies->getState(instance);
	state->setUseCcd(useCcd);
	state->setUseSleep(useSleep);
	state->setUseContactCallback(useContactCallback);
	state->setUseSleepCallback(useSleepCallback);
	if(sleeping) {
		state->sleep();
	}

	state->sleepCount = sleepCount;
	
	return true;
}

bool PfxSnapshotReader::readJoint()
{
	int indent = 0;
	
	Joint joint;

	uint32_t jointFlag = 0;
	uint8_t jointType = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				WorldProperty wp;
				mRigidBodies->getWorldProperty(wp);
				if(joint.stateIndexA < wp.maxInstances &&  joint.stateIndexB < wp.maxInstances) {
					joint.jointFlag = jointFlag;
					joint.jointType = jointType;

					JointProperty jointProperty;
					int j = mRigidBodies->createJoint(jointProperty);
					assert(j >= 0);
					Joint &joint_ = *mRigidBodies->getJoint(j);
					joint_ = joint;
				}
				return true;
			}
		}
		READ_UINT32("jointFlag",jointFlag)
		READ_UINT8("jointType",jointType)
		READ_UINT16("stateIndexA",joint.stateIndexA)
		READ_UINT16("stateIndexB",joint.stateIndexB)
		READ_FLOAT3("linLowerLimit",joint.lowerLimit)
		READ_FLOAT3("angLowerLimit",joint.lowerLimit+3)
		READ_FLOAT3("linUpperLimit",joint.upperLimit)
		READ_FLOAT3("angUpperLimit",joint.upperLimit+3)
		READ_FLOAT("linearDamping",joint.linearDamping)
		READ_FLOAT("angularDamping",joint.angularDamping)
		READ_FLOAT("maxLinearImpulse",joint.maxLinearImpulse)
		READ_FLOAT("maxAngularImpulse",joint.maxAngularImpulse)
		READ_FLOAT("linearImpulseWeight",joint.linearImpulseWeight)
		READ_FLOAT("angularImpulseWeight",joint.angularImpulseWeight)
		READ_FLOAT("linearBias",joint.linearBias)
		READ_FLOAT("angularBias",joint.angularBias)
		READ_FLOAT("breakableLimit",joint.breakableLimit)
		READ_VEC3("anchorA",joint.anchorA)
		READ_VEC3("anchorB",joint.anchorB)
		READ_MATRIX3("frameA",joint.frameA)
		READ_MATRIX3("frameB",joint.frameB)
		READ_MATRIX3("targetFrame",joint.targetFrame)
	}

	return false;
}

uint32_t PfxSnapshotReader::readTriMesh(TriMesh &mesh)
{
	int indent = 0;
	uint32_t id = -1;
	vector<float> verts;
	vector<uint16_t> indices;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				if(!verts.empty() && !indices.empty()) {
					MeshUtil::createTriangleMesh(mesh,&verts[0],verts.size()/3,&indices[0],indices.size());
					return id;
				}
				else {
					return -1;
				}
			}
		}
		else if(isSameTag("Verts")) {
			readVerts(verts);
		}
		else if(isSameTag("Indices")) {
			readIndices(indices);
		}
		READ_UINT32("ID",id)
	}

	return -1;
}

uint32_t PfxSnapshotReader::readConvexMesh(ConvexMesh &mesh)
{
	int indent = 0;
	uint32_t id = -1;
	vector<float> verts;
	vector<uint16_t> indices;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				if(!verts.empty() && !indices.empty()) {
					MeshUtil::createConvexMesh(mesh,&verts[0],verts.size()/3,&indices[0],indices.size());
					return id;
				}
				else {
					return -1;
				}
			}
		}
		else if(isSameTag("Verts")) {
			readVerts(verts);
		}
		else if(isSameTag("Indices")) {
			readIndices(indices);
		}
		READ_UINT32("ID",id)
	}

	return -1;
}

uint32_t PfxSnapshotReader::readLargeMesh(LargeTriMesh &lmesh)
{
	int indent = 0;

	uint32_t id = -1;
	lmesh.numIslands = 0;

	while(getNextTag()) {
		if(isSameTag("{")) {
			indent++;
		}
		else if(isSameTag("}")) {
			indent--;
			if(indent <= 0) {
				return id;
			}
		}
		else if(isSameTag("maxIslands")) {
			uint32_t maxIslands = 0;
			readUint32(maxIslands);
			if(maxIslands > 0) {
				MeshUtil::createLargeTriMesh(lmesh,maxIslands);
			}
		}
		else if(isSameTag("TriMesh")) {
			TriMesh island;
			readTriMesh(island);
			if(island.numFacets > 0 && island.numVerts > 0) {
				MeshUtil::addIslandToLargeTriMesh(lmesh,island);
			}
		}
		READ_UINT32("ID",id)
	}
	
	return -1;
}

void PfxSnapshotReader::readVerts(vector<float> &verts)
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
			Vector3 vtx;
			readVector3(vtx);
			verts.push_back(vtx[0]);
			verts.push_back(vtx[1]);
			verts.push_back(vtx[2]);
		}
	}
}

void PfxSnapshotReader::readIndices(vector<uint16_t> &indices)
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
			uint32_t idx[3];
			readIndex3(idx[0],idx[1],idx[2]);
			indices.push_back(idx[0]);
			indices.push_back(idx[1]);
			indices.push_back(idx[2]);
		}
	}
}

bool PfxSnapshotReader::doImport(const char *filename,
	RigidBodies *rigidbodies,
	map<uint32_t,ConvexMesh>	*meshArray,
	map<uint32_t,LargeTriMesh>	*largeMeshArray
	)
{
	if(!open(filename))
		return false;

	mRigidBodies = rigidbodies;
	mMeshArray = meshArray;
	mLargeMeshArray = largeMeshArray;

	while(getNextTag()) {
		if(isSameTag("ConvexMeshArray")) {
			readConvexMeshArray();
		}
		else if(isSameTag("LargeMeshArray")) {
			readLargeMeshArray();
		}
	}

	reset();

	while(getNextTag()) {
		if(isSameTag("WorldProperty")) {
			WorldProperty wp;
			if(readWorldProperty(wp)) {
				mRigidBodies->setWorldProperty(wp);
				mRigidBodies->reset();
			}
		}
		else if(isSameTag("FormatVersion")) {
			char str[256];
			readString(str,256);
			PRINTF("FormatVersion %s\n",str);
		}
		else if(isSameTag("TrbDynBodyArray")) {
			readTrbDynBodyArray();
		}
		else if(isSameTag("TrbStateArray")) {
			readTrbStateArray();
		}
		else if(isSameTag("JointArray")) {
			readJointArray();
		}
		else if(isSameTag("NonContactPair")) {
			readNonContactPair();
		}
	}

	close();

	return true;
}

bool PfxSnapshotReader::getWorldProperty(const char *filename,
	WorldProperty &worldProperty)
{
	if(!open(filename))
		return false;

	bool checkWorld = false;

	while(getNextTag()) {
		if(isSameTag("WorldProperty")) {
			checkWorld = readWorldProperty(worldProperty);
			break;
		}
	}

	close();

	return checkWorld;
}

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

#include "PfxApp.h"
#include "Physics/RigidBody/Mass.h"
#include "Util/MeshUtil.h"
#include "Util/PfxConvexCreater.h"

const char *getMoveTypeStr(int i)
{
	switch(i) {
		default:
		return "";

		case MoveTypeFixed:
		return "Fixed";

		case MoveTypeActive:
		return "Active";

		case MoveTypeKeyframe:
		return "Keyframe";

		case MoveTypeOneWay:
		return "OneWay";

		case MoveTypeTrigger:
		return "Trigger";
	}
}

const char *getPrimTypeStr(int i)
{
	switch(i) {
		default:
		return "";

		case SPHERE:
		return "Sphere";

		case BOX:
		return "Box";

		case CAPSULE:
		return "Capsule";

		case HEIGHTFIELD:
		return "Height Field";

		case CONVEXMESH:
		return "Convex Mesh";

		case LARGEMESH:
		return "Large Mesh";
	}
}

///////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor

PfxApp::PfxApp()
{
	mSimulationTime = 0.0f;
	mTotalTime = 0.0f;

	LARGE_INTEGER sPerfCountFreq;
	QueryPerformanceFrequency(&sPerfCountFreq);
	mFreq = (float)sPerfCountFreq.QuadPart;

	mWorldProperty.allocatedBuffSize = 10;
	mWorldProperty.pickPower = 20;
	mWorldProperty.lineWidth = 5;
	mWorldProperty.frameRate = 60;

	mDrawFlag = 0;
	mMemSize = mWorldProperty.allocatedBuffSize<<20;
	mRigMemPool = mRayMemPool = NULL;

	for(int i=0;i<256;i++) {
		mColorArray[i] = Vector3(
			(55+rand()%200)/255.0f,
			(55+rand()%200)/255.0f,
			(55+rand()%200)/255.0f);
	}
}

PfxApp::~PfxApp()
{
}

///////////////////////////////////////////////////////////////////////////////
// Memory

void PfxApp::allocatePhysicsMemory()
{
	int rigMemSize = mMemSize * 0.9f;
	int rayMemSize = mMemSize - rigMemSize;
	mRigMemPool = new unsigned char [rigMemSize];
	mRayMemPool = new unsigned char [rayMemSize];
	mRigHeap = new HeapManager(mRigMemPool,rigMemSize);
	mRayHeap = new HeapManager(mRayMemPool,rayMemSize);
}

void PfxApp::deallocatePhysicsMemory()
{
	if(mRigMemPool) delete [] mRigMemPool;
	if(mRayMemPool) delete [] mRayMemPool;
	if(mRigHeap) delete mRigHeap;
	if(mRayHeap) delete mRayHeap;
}

int PfxApp::checkRequiredMemory(WorldProperty &worldProperty)
{
	#define CALCBUF(type,count) (sizeof(type)*count)

	int bufSize = 0;
	int maxPair = (worldProperty.maxInstances*(worldProperty.maxInstances-1)/2);
	int contactTable = PFX_ALIGN128((maxPair+31)/32,sizeof(uint32_t));

	bufSize += CALCBUF(Forces,worldProperty.maxInstances);
	bufSize += CALCBUF(TrbState,worldProperty.maxInstances); // TrbState Buff0
	bufSize += CALCBUF(TrbState,worldProperty.maxInstances); // TrbState Buff0
	bufSize += CALCBUF(TrbState,worldProperty.maxInstances); // TrbState Prev
	bufSize += CALCBUF(TrbDynBody,worldProperty.maxDynBodies);
	bufSize += CALCBUF(CollObject,worldProperty.maxDynBodies);
	bufSize += CALCBUF(CollPrim,worldProperty.maxPrimitives);
	bufSize += CALCBUF(Joint,worldProperty.maxJoints);
	bufSize += CALCBUF(ContactPair,worldProperty.maxContactPairs);
	bufSize += CALCBUF(SortData,worldProperty.maxContactPairs);
	bufSize += CALCBUF(uint32_t,contactTable); // contact table
	bufSize += CALCBUF(uint32_t,worldProperty.maxInstances);	// userData
	bufSize += CALCBUF(uint16_t,worldProperty.maxDynBodies);	// body pool
	bufSize += CALCBUF(uint16_t,worldProperty.maxDynBodies);	// coll pool
	bufSize += CALCBUF(uint16_t,worldProperty.maxPrimitives);	// coll pool
	bufSize += CALCBUF(uint16_t,worldProperty.maxInstances);	// state pool
	bufSize += CALCBUF(uint16_t,worldProperty.maxInstances);	// joint pool

	bufSize *= 1.1f; // add temporay bytes

	return bufSize;

	#undef CALCBUF
}

///////////////////////////////////////////////////////////////////////////////
// Simulation

bool PfxApp::onInitPhysicsEngine()
{
	allocatePhysicsMemory();

	mRigidBodies = new RigidBodies(mRigHeap);
	mRayCast = new RayCast(mRayHeap);
	mRigidBodies->reset();
	return true;
}

void PfxApp::onShutdownPhysicsEngine()
{
	delete mRigidBodies;
	delete mRayCast;

	deallocatePhysicsMemory();
}

void PfxApp::onInitPhysicsScene()
{
	WorldProperty world;
	mRigidBodies->getWorldProperty(world);

	// レイキャストパラメータを設定する
	RaycastProperty rayProp;
	rayProp.maxInstances = world.maxInstances;
	mRayCast->setRaycastProperty(rayProp);
	mRayCast->reset();

	// ピッキング用の固定剛体を作成
	{
		Vector3 boxSize(1.0f);
		float boxMass = 1.0f;

		CollObject *collObj = mRigidBodies->createCollObject();
		assert(collObj);

		collObj->addBox(Box(boxSize), Transform3::identity());
		collObj->finish();

		RigidBodyProperty bodyProperty;
		bodyProperty.mass = boxMass;
		calcInertiaBox(boxSize,boxMass,bodyProperty.inertia);
		bodyProperty.collObject = collObj;
		TrbDynBody *rigidBody = mRigidBodies->createRigidBody(bodyProperty);
		assert(rigidBody);

		InstanceProperty instProperty;
		instProperty.moveType = MoveTypeFixed;
		instProperty.position = Vector3(0,-999,0);
		instProperty.rigidBody = rigidBody;
		mGroundBody = mRigidBodies->createInstance(instProperty);
		assert(mGroundBody >= 0);
	}

	// ピッキング用のジョイントを作成
	{
		JointProperty jointProperty;
		jointProperty.jointType = JointTypeBall;
		jointProperty.parentBody = mGroundBody;
		jointProperty.childBody = 0;
		jointProperty.anchor = Vector3(0.0f);
		int j = mRigidBodies->createJoint(jointProperty);
		assert(j >= 0);
		mPickJoint = mRigidBodies->getJoint(j);
		mPickJoint->maxLinearImpulse = 0.0f;
		mPickJoint->maxAngularImpulse = 0.0f;
		mPickJoint->disableActive();
	}

	// 投擲用の剛体を作成
	{
		float ballRadius = 0.25f;
		float ballMass = 1.0f;

		CollObject *collObj = mRigidBodies->createCollObject();
		assert(collObj);

		collObj->addSphere(Sphere(ballRadius), Transform3::identity());
		collObj->finish();

		RigidBodyProperty bodyProperty;
		bodyProperty.mass = ballMass;
		calcInertiaSphere(ballRadius,ballMass,bodyProperty.inertia);
		bodyProperty.collObject = collObj;
		TrbDynBody *rigidBody = mRigidBodies->createRigidBody(bodyProperty);
		assert(rigidBody);

		InstanceProperty instProperty;
		instProperty.moveType = MoveTypeActive;
		instProperty.position = Vector3(0,-999,0);
		instProperty.rigidBody = rigidBody;
		mBallBody = mRigidBodies->createInstance(instProperty);
		assert(mBallBody >= 0);

		mChargeTime = 0.0f;
	}

	mPickInfo.clear();

	mRigidBodies->setup();
	mRayCast->attachRigidBodies(mRigidBodies);
}

void PfxApp::onReleasePhysicsScene()
{
	bodyIndexArray.clear();
	stateIndexArray.clear();
	jointIndexArray.clear();
	bodyNameArray.clear();
	stateNameArray.clear();
	jointNameArray.clear();

	meshArray.clear();
	for(map<uint32_t,LargeTriMesh>::iterator i=largeMeshArray.begin(); i!=largeMeshArray.end(); i++) {
		MeshUtil::releaseLargeTriMesh(i->second);
	}
	largeMeshArray.clear();

	for(int i=0;i<renderMeshes.size();i++) {
		delete renderMeshes[i].vtx;
		delete renderMeshes[i].nml;
		delete renderMeshes[i].idx;
	}
	renderMeshes.clear();
}

void PfxApp::onWaitSimulation()
{
}

void PfxApp::onUpdateSimulation()
{
}

void PfxApp::onStartSimulation()
{
	LONGLONG t1,t2;
	QueryPerformanceCounter((LARGE_INTEGER*)&t1);

	mRigidBodies->setupSimulate();
	mRigidBodies->ppuSimulate(1.0f/(float)mWorldProperty.frameRate);
	mRigidBodies->finalizeSimulate();

	QueryPerformanceCounter((LARGE_INTEGER*)&t2);
	mSimulationTime = (t2-t1)/mFreq*1000.0f;
}

///////////////////////////////////////////////////////////////////////////////
// Rendering

void PfxApp::onRender()
{
	mRender->drawSceneBegin();

	const float lightPosition[] = {50.0f,50.0f,50.0f,1.0f};
	const float colorDiffuse[] = {1.0f,1.0f,1.0f,1.0f};
	const float colorSpecular[] = {1.0f,1.0f,1.0f,1.0f};

	glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	createProjectionMatrix(mProj);
	glMultMatrixf((GLfloat*)&mProj);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	createModelViewMatrix(mMv);
	glMultMatrixf((GLfloat*)&mMv);

	if(mDrawFlag & DrawFlagSilhouette) {
		glFrontFace(GL_CW);
		glColor4f(0.0f,0.0f,0.0f,1.0f);
		drawObjectsShilhouette();
		glFrontFace(GL_CCW);
	}

	glEnable(GL_LIGHTING) ;   // Enable lighting
	glEnable(GL_LIGHT0) ;     // Light0 ON
	glLightfv(GL_LIGHT0,GL_POSITION,lightPosition);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDisable(GL_TEXTURE_2D);

	glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,colorDiffuse);
	glMaterialfv(GL_FRONT,GL_SPECULAR,colorSpecular);
	glMaterialf(GL_FRONT,GL_SHININESS,20.0f);

	drawObjects();

	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	drawDebugInfo();
	
	mRender->drawSceneEnd();
}

void PfxApp::drawObjectsShilhouette()
{
	for(uint32_t i=0;i<mRigidBodies->getInstanceCount();i++) {
		TrbState	*state = mRigidBodies->getState(i);
		CollObject	*coll = mRigidBodies->getCollObject(i);
		Vector3 halfExtent = coll->getHalf();
		float scale = 1.0f + mWorldProperty.lineWidth*0.01f;
		Transform3 bodyTransform(state->getOrientation(),state->getPosition());

		for(int j=0;j<coll->getNumPrims();j++) {
			CollPrim &prim = coll->getPrim(j);
			Transform3 primTransform = bodyTransform * prim.getObjectRelTransform();
			
			glPushMatrix();
			glMultMatrixf((GLfloat*)&Matrix4(primTransform));

			glScalef(scale,scale,scale);

			switch(prim.getType()) {
				case SPHERE:
				mRenderUtil.drawSphereSilhouette(prim.getPrimDataFloat(0));
				break;

				case BOX:
				mRenderUtil.drawBoxSilhouette(prim.getPrimDataFloat(0),prim.getPrimDataFloat(1),prim.getPrimDataFloat(2));
				break;

				case CAPSULE:
				mRenderUtil.drawCapsuleSilhouette(prim.getPrimDataFloat(0),prim.getPrimDataFloat(1));
				break;

				case CONVEXMESH:
				mRenderUtil.drawMeshSilhouette(renderMeshes[prim.getPrimDataInteger(2)]);
				break;

				default:
				break;
			}

			glPopMatrix();
		}
	}
}

void PfxApp::drawObjects()
{
	const float colorDiffuse1[] = {1.0f,1.0f,1.0f,1.0f};
	const float colorDiffuse2[] = {1.0f,0.7f,0.7f,1.0f};
	const float colorDiffuse3[] = {0.6f,0.6f,0.6f,1.0f};

	for(uint32_t i=0;i<mRigidBodies->getInstanceCount();i++) {
		TrbState	*state = mRigidBodies->getState(i);
		CollObject	*coll = mRigidBodies->getCollObject(i);
		Transform3 bodyTransform(state->getOrientation(),state->getPosition());

		if(mPickInfo.ray.contactFlag && mPickInfo.ray.contactInstance == i) {
			glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,colorDiffuse2);
		}
		else {
			if(state->getMoveTypeBits()&((1<<MoveTypeKeyframe)|(1<<MoveTypeFixed))) {
				glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,colorDiffuse3);
			}
			else {
				glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,colorDiffuse1);
			}
		}

		for(int j=0;j<coll->getNumPrims();j++) {
			CollPrim &prim = coll->getPrim(j);
			Transform3 primTransform = bodyTransform * prim.getObjectRelTransform();
			
			glPushMatrix();
			glMultMatrixf((GLfloat*)&Matrix4(primTransform));
			
			switch(prim.getType()) {
				case SPHERE:
				mRenderUtil.drawSphere(prim.getPrimDataFloat(0));
				break;

				case BOX:
				mRenderUtil.drawBox(prim.getPrimDataFloat(0),prim.getPrimDataFloat(1),prim.getPrimDataFloat(2));
				break;

				case CAPSULE:
				mRenderUtil.drawCapsule(prim.getPrimDataFloat(0),prim.getPrimDataFloat(1));
				break;

				case CONVEXMESH:
				mRenderUtil.drawMesh(renderMeshes[prim.getPrimDataInteger(2)]);
				break;

				case LARGEMESH:
				mRenderUtil.drawMesh(renderMeshes[prim.getPrimDataInteger(2)]);
				if(mDrawFlag & DrawFlagIslandsAABB) {
					LargeTriMesh *mesh = prim.getLargeMesh();
					if(mesh) {
						glDisable(GL_LIGHTING);
#if 0
						for(int l=0;l<mesh->numIslands;l++) {
							glColor3fv((float*)&mColorArray[l]);
							PfxAABB16 aabb = mesh->aabbList[l];
							Vector3 aabbMin = mesh->getWorldPosition(VecInt3((int32_t)XMin(aabb),(int32_t)YMin(aabb),(int32_t)ZMin(aabb)));
							Vector3 aabbMax = mesh->getWorldPosition(VecInt3((int32_t)XMax(aabb),(int32_t)YMax(aabb),(int32_t)ZMax(aabb)));
							Vector3 center = (aabbMin + aabbMax) * 0.5f;
							Vector3 half = (aabbMax - aabbMin) * 0.5f;
							mRenderUtil.drawFrame((float*)&center,(float*)&half);
						}
#else
						glBegin(GL_LINES);
						for(int l=0;l<mesh->numIslands;l++) {
							TriMesh &island = mesh->islands[l];
							for(int e=0;e<island.numEdges;e++) {
								MeshEdge &edge = island.edges[e];
								if(edge.angle == EDGE_CONVEX) {
									glColor3fv((float*)&mColorArray[l]);
								}
								else {
									glColor3f(0,0,0);
								}
								uint8_t vi1 = edge.vertIndex[0];
								uint8_t vi2 = edge.vertIndex[1];
								glVertex3fv((float*)&island.verts[vi1]);
								glVertex3fv((float*)&island.verts[vi2]);
							}
						}
						glEnd();
#endif
						glEnable(GL_LIGHTING);
					}
				}
				break;

				default:
				break;
			}

			glPopMatrix();
		}
	}
}

void PfxApp::drawDebugInfo()
{
	for(uint32_t i=0;i<mRigidBodies->getInstanceCount();i++) {
		TrbState	*state = mRigidBodies->getState(i);
		TrbDynBody  *body = mRigidBodies->getTrbDynBody(i);
		CollObject	*coll = body->getCollObject();
		
		Vector3 center(state->center[0],state->center[1],state->center[2]); 
		Vector3 half(state->half[0],state->half[1],state->half[2]);
		Vector3 pos(state->getPosition());
		Matrix3 rot(state->getOrientation());
		
		// AABB
		if(mDrawFlag & DrawFlagAABB) {
			glEnable(GL_DEPTH_TEST);
			if(state->isAsleep())
				glColor4f(0.5f,0,0,1);
			else
				glColor4f(1,0,0,1);
			mRenderUtil.drawFrame((float*)&center,(float*)&half);
			glDisable(GL_DEPTH_TEST);

			glColor4f(0,0,1,1);
			Vector3 p = pos + state->getLinearVelocity();
			mRenderUtil.drawLine((float*)&pos,(float*)&p);
		}
		
		// Local Axis
		if(mDrawFlag & DrawFlagLocalAxis) {
			float len = 0.15f*(half[0]+half[1]+half[2]);
			Vector3 pX = pos + len*rot.getCol(0);
			Vector3 pY = pos + len*rot.getCol(1);
			Vector3 pZ = pos + len*rot.getCol(2);
			glColor4f(1,0,0,1);
			mRenderUtil.drawLine((float*)&pos,(float*)&pX);
			glColor4f(0,1,0,1);
			mRenderUtil.drawLine((float*)&pos,(float*)&pY);
			glColor4f(0,0,1,1);
			mRenderUtil.drawLine((float*)&pos,(float*)&pZ);
		}

		// Inertia Tensor
		if(mDrawFlag & DrawFlagInertia) {
			Matrix3 inertia = body->getBodyInertia();
			Vector3 pX = pos + rot * inertia.getCol(0);
			Vector3 pY = pos + rot * inertia.getCol(1);
			Vector3 pZ = pos + rot * inertia.getCol(2);
			glColor4f(0,1,0,1);
			mRenderUtil.drawLine((float*)&pos,(float*)&pX);
			mRenderUtil.drawLine((float*)&pos,(float*)&pY);
			mRenderUtil.drawLine((float*)&pos,(float*)&pZ);
		}
	}

	if(mDrawFlag & DrawFlagJoint) {
		for(uint32_t i=0;i<mRigidBodies->getJointCount();i++) {
			Joint &joint = *mRigidBodies->getJoint(i);
			if(!joint.isActive()) continue;
			TrbState *stateA = mRigidBodies->getState(joint.stateIndexA);
			TrbState *stateB = mRigidBodies->getState(joint.stateIndexB);
			Matrix3 wRotA(stateA->getOrientation());
			Matrix3 wRotB(stateB->getOrientation());
			Vector3 wAnchorA = stateA->getPosition() + wRotA * joint.anchorA;
			Vector3 wAnchorB = stateB->getPosition() + wRotB * joint.anchorB;
			Matrix3 wFrameA = wRotA * joint.frameA;
			Matrix3 wFrameB = wRotB * joint.frameB;
			Matrix4 wTransA(wFrameA,wAnchorA);
			Matrix4 wTransB(wFrameB,wAnchorB);
			Vector3 halfVec = minPerElem(
				Vector3(stateA->half[0],stateA->half[1],stateA->half[2]),
				Vector3(stateB->half[0],stateB->half[1],stateB->half[2]));
			float len = 0.2f*length(halfVec);

			glPushMatrix();
			glMultMatrixf((GLfloat*)&wTransA);
			mRenderUtil.drawAxis(len);
			glPopMatrix();

			glPushMatrix();
			glMultMatrixf((GLfloat*)&wTransB);
			mRenderUtil.drawAxis(len);
			glPopMatrix();
		}
	}

	if(mDrawFlag & DrawFlagContact) {
		for(uint32_t i=0;i<mRigidBodies->getContactCount();i++) {
			ContactPair &contact = *mRigidBodies->getContactPair(i);
			for(uint32_t c=0;c<contact.numContacts;c++) {
				ContactPoint &cp = contact.contactPoints[c];

				Vector3 pA = mRigidBodies->getWorldPosition(contact.stateIndex[0],cp.getLocalPointA());
				Vector3 pA2 = pA + 0.1f * cp.getNormal();
				glColor4f(1,0,0,1);
				//mRenderUtil.drawStar((float*)&pA,0.1f);
				mRenderUtil.drawLine((float*)&pA,(float*)&pA2);

				Vector3 pB = mRigidBodies->getWorldPosition(contact.stateIndex[1],cp.getLocalPointB());
				Vector3 pB2 = pB + 0.1f * cp.getNormal();
				glColor4f(0,0,1,1);
				//mRenderUtil.drawStar((float*)&pB,0.1f);
				mRenderUtil.drawLine((float*)&pB,(float*)&pB2);

				glColor4f(1,0,0,1);
				mRenderUtil.drawStar((float*)&pA,0.02f);

				glColor4f(0,0,1,1);
				mRenderUtil.drawStar((float*)&pB,0.02f);
			}
		}
	}

	if(mDrawFlag & DrawFlagPerf) {
		glColor3f(1,1,1);
		Vector3 v(-mRender->getWidth()*0.5f+2.0f,-mRender->getHeight()*0.5f+2.0f,0.9998f);
		screenToWorld(v);
		glRasterPos3fv((float*)&v);
		glPrint("Total %3.2fms Simulation %3.2fms",mTotalTime,mSimulationTime);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Picking

void PfxApp::pickClick(int x,int y,ClickMode mode)
{
	// カーソル位置をワールド空間に投影
	Vector3 wp1 = Vector3(x,y,0.0f);
	Vector3 wp2	= Vector3(x,y,1.0f);
	screenToWorld(wp1);
	screenToWorld(wp2);
	
	// ピッキング
	Ray ray;
	ray.startPos = wp1;
	ray.endPos = wp2;
	mRayCast->setupRayCast();
	mRayCast->ppuRayCast(&ray,1);
	mRayCast->finalizeRayCast();

	mPickInfo.ray = ray;

	if(ray.contactFlag) {
		Vector3 sp = ray.contactPoint;
		worldToScreen(sp);
		mPickZ = sp[2];

		TrbState *state = mRigidBodies->getState(ray.contactInstance);
		TrbDynBody *body = mRigidBodies->getTrbDynBody(ray.contactInstance);
		CollObject *coll = body->getCollObject();
		string stateName;
		map<uint32_t,string>::iterator iState = stateNameArray.find(ray.contactInstance);

		if(iState != stateNameArray.end())
			stateName = iState->second;

		mPickInfo.name = stateName;
		mPickInfo.moveType = getMoveTypeStr(state->getMoveType());
		mPickInfo.primType = "";
		int maxPrimStr = PFX_MIN(10,coll->getNumPrims());
		for(int p=0;p<maxPrimStr;p++) {
			if(p>0) mPickInfo.primType += ",";
			mPickInfo.primType += getPrimTypeStr(coll->getPrim(p).getType());
		}
		if(maxPrimStr < coll->getNumPrims()) {
			mPickInfo.primType += ",...";
		}
		
		mPickInfo.mass = body->getMass();
		mPickInfo.inertia = body->getBodyInertia();
		mPickInfo.friction = body->getFriction();
		mPickInfo.resitution = body->getElasticity();
		mPickInfo.aabbHalf = Vector3(state->half[0],state->half[1],state->half[2]);
		mPickInfo.position = state->getPosition();
		mPickInfo.orentation = state->getOrientation();
		mPickInfo.linearVelocity = state->getLinearVelocity();
		mPickInfo.angularVelocity = state->getAngularVelocity();

		if(mode == ClickModePick) {
			mPickJoint->stateIndexB = ray.contactInstance;
			mPickJoint->anchorA = mRigidBodies->getLocalPosition(mPickJoint->stateIndexA,ray.contactPoint);
			mPickJoint->anchorB = mRigidBodies->getLocalPosition(mPickJoint->stateIndexB,ray.contactPoint);
			mPickJoint->maxLinearImpulse = mPickInfo.mass * 0.1f * (float)mWorldProperty.pickPower;
			mPickJoint->maxAngularImpulse = mPickInfo.mass * 0.1f * (float)mWorldProperty.pickPower;
			mPickJoint->enableActive();

			if(ray.subData.type == SubData::SubDataFacetLocal) {
				PRINTF("island %d facet %d\n",ray.subData.getIslandIndex(),ray.subData.getFacetIndex());
			}
		}
		else if(mode == ClickModeView) {
			mViewTarget = ray.contactPoint;
			Vector3 aabb(state->half[0],state->half[1],state->half[2]);
			mViewRadius = length(aabb) * 3.0f;
		}
	}
}

void PfxApp::pickOver(int x,int y)
{
	// ジョイントの接続位置を更新する
	if(mPickJoint->isActive()) {
		Vector3 worldProp = Vector3(x,y,mPickZ);
		screenToWorld(worldProp);
		mPickJoint->anchorA = mRigidBodies->getLocalPosition(mPickJoint->stateIndexA,worldProp);
		if(mRigidBodies->isAsleep(mPickJoint->stateIndexB))
			mRigidBodies->wakeup(mPickJoint->stateIndexB);
	}
}

void PfxApp::pickLeave(int x,int y)
{
	if(mPickJoint->isActive()) {
		mPickJoint->disableActive();
	}
}

bool PfxApp::getPickInfo(string &msg)
{
	msg.clear();
	if(mPickInfo.ray.contactFlag) {
		char buf[256];

		sprintf(buf,"Name : \"%s\"(%d) MoveType : \"%s\"\r\n",
			mPickInfo.name.c_str(),mPickInfo.ray.contactInstance,mPickInfo.moveType.c_str());
		msg += buf;

		sprintf(buf,"AABB : %.2f,%.2f,%.2f Shape : \"%s\"\r\n",
			(float)mPickInfo.aabbHalf[0]*2.0f,(float)mPickInfo.aabbHalf[1]*2.0f,(float)mPickInfo.aabbHalf[2]*2.0f,
			mPickInfo.primType.c_str());
		msg += buf;

		sprintf(buf,"Friction : %.2f Restitution : %.2f\r\n",
			mPickInfo.friction,mPickInfo.resitution);
		msg += buf;

		sprintf(buf,"Mass : %.2f Inertia : %.2f,%.2f,%.2f | %.2f,%.2f,%.2f | %.2f,%.2f,%.2f\r\n",
			mPickInfo.mass,
			(float)mPickInfo.inertia.getCol0()[0],(float)mPickInfo.inertia.getCol1()[0],(float)mPickInfo.inertia.getCol2()[0],
			(float)mPickInfo.inertia.getCol0()[1],(float)mPickInfo.inertia.getCol1()[1],(float)mPickInfo.inertia.getCol2()[1],
			(float)mPickInfo.inertia.getCol0()[2],(float)mPickInfo.inertia.getCol1()[2],(float)mPickInfo.inertia.getCol2()[2]);
		msg += buf;

		return true;
	}

	return false;
}

void PfxApp::fire()
{
	Vector3 vel = -normalize(mViewPos) * mChargeTime * THROW_POWER;

	mRigidBodies->setPosition(mBallBody,mViewTarget+mViewPos);
	mRigidBodies->setOrientation(mBallBody,Quat::identity());
	mRigidBodies->setVelocity(mBallBody,vel);
	mRigidBodies->setAngularVelocity(mBallBody,Vector3(0.0f));
	mRigidBodies->setMoveType(mBallBody,MoveTypeActive);
}

void PfxApp::deleteSelection()
{
	if(mPickInfo.ray.contactFlag) {
		uint32_t inst = mPickInfo.ray.contactInstance;
		mRigidBodies->deleteInstance(inst);
		for(int i=0;i<mRigidBodies->getJointCount();i++) {
			Joint *joint = mRigidBodies->getJoint(i);
			if(joint->stateIndexA == inst || joint->stateIndexB == inst) {
				joint->disableActive();
			}
		}

		mPickInfo.ray.contactFlag = false;
	}
}

///////////////////////////////////////////////////////////////////////////////
// World Property

void PfxApp::getWorldProperty(PfxAppWorldProperty &wp)
{
	WorldProperty rwp;
	mRigidBodies->getWorldProperty(rwp);
	mWorldProperty.maxInstances = rwp.maxInstances;
	mWorldProperty.maxDynBodies = rwp.maxDynBodies;
	mWorldProperty.maxJoints = rwp.maxJoints;
	mWorldProperty.maxContactPairs = rwp.maxContactPairs;
	mWorldProperty.rigBufferSize = mRigidBodies->calcBufSize();
	mWorldProperty.rayBufferSize = mRayCast->calcBufSize();
	mWorldProperty.subStepCount = mRigidBodies->getSubStepCount();
	mWorldProperty.contactIteration = rwp.contactIteration;
	mWorldProperty.jointIteration = rwp.jointIteration;
	mWorldProperty.sleepEnable = rwp.sleepEnable;
	wp = mWorldProperty;
}

void PfxApp::setWorldProperty(const PfxAppWorldProperty &wp)
{
	mRigidBodies->setSubStepCount(wp.subStepCount);
	mRigidBodies->setContactIteration(wp.contactIteration);
	mRigidBodies->setJointIteration(wp.jointIteration);
	mRigidBodies->setSleepEnable(wp.sleepEnable);
	mWorldProperty.allocatedBuffSize = wp.allocatedBuffSize;
	mWorldProperty.pickPower = wp.pickPower;
	mWorldProperty.lineWidth = wp.lineWidth;
	mWorldProperty.frameRate = wp.frameRate;
}

///////////////////////////////////////////////////////////////////////////////
// Update

bool PfxApp::onUpdate()
{
	static LONGLONG t1=0;
	LONGLONG t2;
	QueryPerformanceCounter((LARGE_INTEGER*)&t2);
	mTotalTime = (t2-t1)/mFreq*1000.0f;
	t1 = t2;

	static int chkPressId = 0;
	static bool chkPress[2] = {false,false};
	chkPress[chkPressId] = (GetKeyState('A')&0x80)?true:false;

	if(chkPress[chkPressId]&&chkPress[1-chkPressId]) {
		if(mChargeTime < MAX_CHARGE)
			mChargeTime += 2.0f/(float)mWorldProperty.frameRate;
	}
	else if(chkPress[1-chkPressId]&&!chkPress[chkPressId]) {
		//fire();
		mChargeTime = 0.0f;
	}
	else {
		mChargeTime = 0.0f;
	}

	chkPressId = 1-chkPressId;

	return CommonApp::onUpdate();
}

///////////////////////////////////////////////////////////////////////////////
// Convert to render mesh

void PfxApp::convertToRenderMesh(RenderUtil::Mesh &rmesh,ConvexMesh &mesh)
{
	rmesh.numVerts = mesh.numVerts;
	rmesh.numIndices = mesh.numIndices;
	rmesh.vtx = new float[rmesh.numVerts*3];
	rmesh.nml = new float[rmesh.numVerts*3];
	rmesh.idx = new unsigned short[rmesh.numIndices];

	for(uint32_t i=0;i<mesh.numVerts;i++) {
		rmesh.vtx[i*3  ] = mesh.verts[i].getX();
		rmesh.vtx[i*3+1] = mesh.verts[i].getY();
		rmesh.vtx[i*3+2] = mesh.verts[i].getZ();
	}

	for(uint32_t i=0;i<mesh.numIndices;i++) {
		rmesh.idx[i] = mesh.indices[i];
	}

	mRenderUtil.calcNormal(rmesh);
}

void PfxApp::convertToRenderMesh(RenderUtil::Mesh &rmesh,LargeTriMesh &mesh)
{
	uint32_t numVerts = 0;
	uint32_t numIndices = 0;

	for(uint32_t i=0;i<mesh.numIslands;i++) {
		numVerts += mesh.islands[i].numVerts;
		numIndices += mesh.islands[i].numFacets*3;
	}
	
	rmesh.numVerts = numVerts;
	rmesh.numIndices = numIndices;
	rmesh.vtx = new float[numVerts*3];
	rmesh.nml = new float[numVerts*3];
	rmesh.idx = new unsigned short[numIndices];

	uint32_t vid = 0;
	uint32_t fid = 0;

	for(uint32_t i=0;i<mesh.numIslands;i++) {
		TriMesh &island = mesh.islands[i];

		for(uint32_t j=0;j<island.numVerts;j++) {
			rmesh.vtx[(vid+j)*3  ] = island.verts[j].getX();
			rmesh.vtx[(vid+j)*3+1] = island.verts[j].getY();
			rmesh.vtx[(vid+j)*3+2] = island.verts[j].getZ();
		}

		for(uint32_t j=0;j<island.numFacets;j++) {
			rmesh.idx[(fid+j)*3  ] = vid + island.facets[j].vertIndices[0];
			rmesh.idx[(fid+j)*3+1] = vid + island.facets[j].vertIndices[1];
			rmesh.idx[(fid+j)*3+2] = vid + island.facets[j].vertIndices[2];
		}

		vid+=island.numVerts;
		fid+=island.numFacets;
	}

	mRenderUtil.calcNormal(rmesh);
}

///////////////////////////////////////////////////////////////////////////////
// Import Scene from Pfx file

void PfxApp::importSceneFromPfx(const char *filename)
{
	onReleasePhysicsScene();
	
	// シーンファイル読み込み
	if(!mPfxReader.doImport(filename)) {
		PRINTF("Can't load a scene file\n");
	}
	
	// 剛体シーンを作成する
	createSceneFromFile();

	onInitPhysicsScene();
}

///////////////////////////////////////////////////////////////////////////////
// Import Scene from Snapshot file

void PfxApp::importSceneFromSnapshot(const char *filename)
{
	onReleasePhysicsScene();
	
	// メモリチェック
	WorldProperty wp;
	if(mSnpReader.getWorldProperty(filename,wp)) {
		int requiredBytes = checkRequiredMemory(wp);
		if(requiredBytes > mMemSize * 0.9f) {
			requiredBytes = requiredBytes * 1.1f;
			requiredBytes = ((requiredBytes + (1<<20)-1)>>20)<<20;
			mMemSize = requiredBytes;
			mWorldProperty.allocatedBuffSize = mMemSize>>20;
			onShutdownPhysicsEngine();
			onInitPhysicsEngine();
		}
	}

	// シーンファイル読み込み
	if(!mSnpReader.doImport(filename,mRigidBodies,&meshArray,&largeMeshArray)) {
		PRINTF("Can't load a scene file\n");
	}
	
	// メッシュを描画オブジェクトと結びつける
	for(uint32_t i=0;i<mRigidBodies->getTrbDynBodyCount();i++) {
		TrbDynBody *body = mRigidBodies->getTrbDynBodyByIndex(i);
		CollObject *coll = body->getCollObject();
		for(uint32_t j=0;j<coll->getNumPrims();j++) {
			CollPrim &prim = coll->getPrim(j);
			if(prim.getType() == CONVEXMESH) {
				ConvexMesh *mesh = prim.getConvexMesh();
				RenderUtil::Mesh rmesh;
				convertToRenderMesh(rmesh,*mesh);
				renderMeshes.push_back(rmesh);
				prim.setPrimDataInteger(2,renderMeshes.size()-1);
			}
			else if(prim.getType() == LARGEMESH) {
				LargeTriMesh *mesh = prim.getLargeMesh();
				RenderUtil::Mesh rmesh;
				convertToRenderMesh(rmesh,*mesh);
				renderMeshes.push_back(rmesh);
				prim.setPrimDataInteger(2,renderMeshes.size()-1);
#if 0 // try to recreate islands
				if(mesh->numIslands < 200) {
					MeshUtil::releaseLargeTriMesh(*mesh);
					MeshUtil::LargeTriMeshConfig config;
					MeshUtil::autoGenerateLargeTriMesh(config,*mesh,
						rmesh.vtx,rmesh.numVerts,
						rmesh.idx,rmesh.numIndices);
				}
#endif
			}
		}
	}

	onInitPhysicsScene();
}

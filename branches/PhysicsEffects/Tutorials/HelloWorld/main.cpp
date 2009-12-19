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
#include "Physics/RigidBody/RigidBodies.h"
#include "Physics/RigidBody/Mass.h"

RigidBodies *mRigidBodies=0;

TrbDynBody* createRigidBodyBox(const Vector3 &boxSize,const float boxMass)
{
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
	
	return rigidBody;
}


void createScene1()
{
	int size = 8;
	const float cubeSize = 1.0f;
	float spacing = cubeSize;
	Vector3 pos(0.0f, cubeSize * 2, 0.0f);
	float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;

	for(int k=0;k<47;k++) {
		for(int j=0;j<size;j++) {
			pos[2] = offset + (float)j * (cubeSize * 2.0f + spacing);
			for(int i=0;i<size;i++) {
				pos[0] = offset + (float)i * (cubeSize * 2.0f + spacing);

				Vector3 boxSize = mulPerElem(Vector3(1.0f),Vector3(cubeSize));

				TrbDynBody *rigidBody = createRigidBodyBox(boxSize,2.0f);

				InstanceProperty biproperty;
				biproperty.moveType = MoveTypeActive;
				biproperty.rigidBody = rigidBody;
				biproperty.position = pos;

				int instance = mRigidBodies->createInstance(biproperty);
				assert(instance >= 0);
			}
		}
		offset -= 0.05f * spacing * (size-1);
		spacing *= 1.01f;
		pos[1] += (cubeSize * 2.0f + spacing);
	}
}


int createGround()
{
	Vector3 groundSize(150.0f, 2.0f, 150.0f);
	TrbDynBody *rigidBody = createRigidBodyBox(groundSize,5.0f);

	InstanceProperty biproperty;
	biproperty.moveType = MoveTypeFixed;
	biproperty.rigidBody = rigidBody;
	biproperty.position = Vector3(0.f,0.f,0.f);
	int instance = mRigidBodies->createInstance(biproperty);
	return 0;
}


#define HEAP_BYTES_RB (35*1024*1024)
unsigned char memPoolRB[HEAP_BYTES_RB] __attribute__ ((aligned(128)));
HeapManager gPoolRB(memPoolRB,HEAP_BYTES_RB);
#define TIMESTEP 1.f/60.f

int main( int argc, char *argv[])
{
#ifdef _WIN32
	mRigidBodies = new RigidBodies(&gPoolRB);
#else
	mRigidBodies = new RigidBodies(0,0,&gPoolRB);
#endif


	// ワールドパラメータを設定する
	WorldProperty worldProp;
	worldProp.maxDynBodies = 3500;
	worldProp.maxInstances = 3500;
	worldProp.maxJoints = 2000;
	worldProp.maxSprings = 0;
	worldProp.worldCenter = Vector3(0,90,0);
	worldProp.worldExtent = Vector3(500,100,500);
	worldProp.maxContactPairs = 20000;
	worldProp.ccdEnable = true; // CCDを扱えるようにセット
	mRigidBodies->setWorldProperty(worldProp);
	mRigidBodies->reset();

	int mGroundBody = createGround();


	createScene1();

	mRigidBodies->setup();

	for (int i=0;i<130;i++)
	{
	mRigidBodies->setupSimulate();
	mRigidBodies->ppuSimulate(TIMESTEP);
	mRigidBodies->finalizeSimulate();
	}
	mRigidBodies->saveSnapshot("world_win32.txt");

	return 0;
}

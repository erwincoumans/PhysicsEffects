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
#include <vectormath_aos.h>

#include "Physics/RigidBody/common/Contact.h"
#include "Physics/RigidBody/common/CollObject.h"

#include "trianglesBoxContacts.h"
#include "trianglesSphereContacts.h"
#include "trianglesCapsuleContacts.h"
#include "trianglesConvexContacts.h"

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"
#include "Physics/RigidBody/common/SubData.h"


using namespace Vectormath::Aos;

#define SET_CONTACT_POINT(contactPoint,dist,nml,pntA,trnsA,primA,pntB,trnsB,primB) \
	contactPoint.distance = dist;\
	contactPoint.setNormal(nml);\
	contactPoint.setA(pntA, trnsA, primA);\
	contactPoint.setB(pntB, trnsB, primB);\
	contactPoint.subData.type = SubData::SubDataNone;

///////////////////////////////////////////////////////////////////////////////
// largeMesh x Shape


int primContactsShapeLargeMesh(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	int numContacts = 0;

	Transform3 transformBA;
	Matrix3 matrixBA;
	Vector3 offsetBA;

	// Aローカル→Bローカルへの変換
	transformBA = orthoInverse(primTransformB) * primTransformA;
	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();
	
	// -----------------------------------------------------
	// LargeTriMeshに含まれるTriMeshのAABBと凸体のAABBを判定し、
	// 交差するものを個別に衝突判定する。※LargeMesh座標系
	
	Vector3 shapeHalf(0.0f);
	Vector3 shapeCenter = offsetBA;

	switch(primA.getType()) {
		case SPHERE:
		shapeHalf = primA.getSphere().GetAABB();
		break;
		
		case CAPSULE:
		shapeHalf = primA.getCapsule().GetAABB(matrixBA.getCol0());
		break;
		
		case BOX:
		shapeHalf = primA.getBox().GetAABB(matrixBA);
		break;
		
		case CONVEXMESH:
		shapeHalf = primA.getConvexMesh()->getAABB(matrixBA);
		break;

		default:
		break;
	}

	// -----------------------------------------------------
	// アイランドとの衝突判定

	LargeTriMesh *largeMesh = primB.getLargeMesh();

	VecInt3 aabbMinL = largeMesh->getLocalPosition((shapeCenter-shapeHalf));
	VecInt3 aabbMaxL = largeMesh->getLocalPosition((shapeCenter+shapeHalf));

	for(uint8_t i=0;i<largeMesh->numIslands;i++) {
		// AABBチェック
		PfxAABB16 aabbB = largeMesh->aabbList[i];
		if(aabbMaxL.getX() < XMin(aabbB) || aabbMinL.getX() > XMax(aabbB)) continue;
		if(aabbMaxL.getY() < YMin(aabbB) || aabbMinL.getY() > YMax(aabbB)) continue;
		if(aabbMaxL.getZ() < ZMin(aabbB) || aabbMinL.getZ() > ZMax(aabbB)) continue;

		TriMesh *island = &largeMesh->islands[i];
		Vector3 testNormal[4];
		Point3 pointA[4],pointB[4];
		SubData subData[4];
		float distance[4];
		int numNewCp = 0;

		// 衝突判定
		switch(primA.getType()) {
			case SPHERE:
			numNewCp = trianglesSphereContacts( testNormal, subData, pointB, pointA, distance,
										   island, primTransformB,
										   primA.getSphere(), primTransformA, objsInContactDist );
			break;

			case CAPSULE:
			numNewCp = trianglesCapsuleContacts( testNormal, subData, pointB, pointA, distance,
										   island, primTransformB,
										   primA.getCapsule(), primTransformA, objsInContactDist );
			break;
			
			case BOX:
			numNewCp = trianglesBoxContacts( testNormal, subData, pointB, pointA, distance,
										   island, primTransformB,
										   primA.getBox(), primTransformA, objsInContactDist );
			break;
			
			case CONVEXMESH:
			numNewCp = trianglesConvexContacts( testNormal, subData, pointB, pointA, distance,
											island, primTransformB,
											primA.getConvexMesh(), primTransformA, objsInContactDist );
			break;

			default:
			break;
		}
		
		// 衝突点を追加
		for(int k=0;k<numNewCp;k++) {
			if(distance[k] >= objsInContactDist) continue;
			
			Vector3 newCp(relTransformA*pointA[k]);
			
			int replaceId = findNearestContactPoint(cp,numContacts,newCp);
			
			if(replaceId < 0) {
				if(numContacts < 4) {
					replaceId = numContacts++;
				}
				else {
					replaceId = sort4ContactPoints(cp,newCp,distance[k]);
				}
			}

			SET_CONTACT_POINT(cp[replaceId],distance[k],-testNormal[k],
				pointA[k], relTransformA, primIndexA,
				pointB[k], relTransformB, primIndexB);
			cp[replaceId].subData = subData[k];
			cp[replaceId].subData.setIslandIndex(i);
		}
	}

	return numContacts;
}

int primContactsLargeMeshShape(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	int numContacts = 0;

	Transform3 transformAB;
	Matrix3 matrixAB;
	Vector3 offsetAB;
	
	// Bローカル→Aローカルへの変換
	transformAB = orthoInverse(primTransformA) * primTransformB;
	matrixAB = transformAB.getUpper3x3();
	offsetAB = transformAB.getTranslation();
	
	// -----------------------------------------------------
	// LargeTriMeshに含まれるTriMeshのAABBと凸体のAABBを判定し、
	// 交差するものを個別に衝突判定する。※LargeMesh座標系
	
	Vector3 shapeHalf(0.0f);
	Vector3 shapeCenter = offsetAB;

	switch(primB.getType()) {
		case SPHERE:
		shapeHalf = primB.getSphere().GetAABB();
		break;
		
		case CAPSULE:
		shapeHalf = primB.getCapsule().GetAABB(matrixAB.getCol0());
		break;
		
		case BOX:
		shapeHalf = primB.getBox().GetAABB(matrixAB);
		break;
		
		case CONVEXMESH:
		shapeHalf = primB.getConvexMesh()->getAABB(matrixAB);
		break;

		default:
		break;
	}

	// -----------------------------------------------------
	// アイランドとの衝突判定

	LargeTriMesh *largeMesh = primA.getLargeMesh();

	VecInt3 aabbMinL = largeMesh->getLocalPosition((shapeCenter-shapeHalf));
	VecInt3 aabbMaxL = largeMesh->getLocalPosition((shapeCenter+shapeHalf));
	
	for(uint8_t i=0;i<largeMesh->numIslands;i++) {
		// AABBチェック
		PfxAABB16 aabbB = largeMesh->aabbList[i];
		if(aabbMaxL.getX() < XMin(aabbB) || aabbMinL.getX() > XMax(aabbB)) continue;
		if(aabbMaxL.getY() < YMin(aabbB) || aabbMinL.getY() > YMax(aabbB)) continue;
		if(aabbMaxL.getZ() < ZMin(aabbB) || aabbMinL.getZ() > ZMax(aabbB)) continue;

		TriMesh *island = &largeMesh->islands[i];
		Vector3 testNormal[4];
		Point3 pointA[4],pointB[4];
		SubData subData[4];
		float distance[4];
		int numNewCp = 0;

		// 衝突判定
		switch(primB.getType()) {
			case SPHERE:
			numNewCp = trianglesSphereContacts( testNormal, subData, pointA, pointB, distance,
										   island, primTransformA,
										   primB.getSphere(), primTransformB, objsInContactDist );
			break;

			case CAPSULE:
			numNewCp = trianglesCapsuleContacts( testNormal, subData, pointA, pointB, distance,
										   island, primTransformA,
										   primB.getCapsule(), primTransformB, objsInContactDist );
			break;
			
			case BOX:
			numNewCp = trianglesBoxContacts( testNormal, subData, pointA, pointB, distance,
										   island, primTransformA,
										   primB.getBox(), primTransformB, objsInContactDist );
			break;
			
			case CONVEXMESH:
			numNewCp = trianglesConvexContacts( testNormal, subData, pointA, pointB, distance,
											island, primTransformA,
											primB.getConvexMesh(), primTransformB, objsInContactDist);
			break;

			default:
			break;
		}

		// 衝突点を追加
		for(int k=0;k<numNewCp;k++) {
			if(distance[k] >= objsInContactDist) continue;
			
			Vector3 newCp(relTransformA*pointA[k]);
			
			int replaceId = findNearestContactPoint(cp,numContacts,newCp);
			
			if(replaceId < 0) {
				if(numContacts < 4) {
					replaceId = numContacts++;
				}
				else {
					replaceId = sort4ContactPoints(cp,newCp,distance[k]);
				}
			}

			SET_CONTACT_POINT(cp[replaceId],distance[k],testNormal[k],
				pointA[k], relTransformA, primIndexA,
				pointB[k], relTransformB, primIndexB);
			cp[replaceId].subData = subData[k];
			cp[replaceId].subData.setIslandIndex(i);
		}
	}

	return numContacts;
}



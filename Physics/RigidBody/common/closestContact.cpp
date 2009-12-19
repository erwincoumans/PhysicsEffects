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

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/Base/SimpleStack.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"
#include "intersectFunction.h"
#include "SubData.h"
#include "closestContact.h"
#include "boxBoxDistance.h"
#include "boxCapsuleDistance.h"
#include "boxSphereDistance.h"
#include "capsuleCapsuleDistance.h"
#include "capsuleSphereDistance.h"
#include "sphereSphereDistance.h"
#include "trianglesBoxContacts.h"
#include "trianglesSphereContacts.h"
#include "trianglesCapsuleContacts.h"


///////////////////////////////////////////////////////////////////////////////
// ContactPair Function Table

#define PRIMCONTACTFUNC(funcName) \
int funcName(\
				ContactPoint *cp,\
				const CollPrim & primA,const Transform3 &primTransformA,const Transform3 &relTransformA,int primIndexA,\
				const CollPrim & primB,const Transform3 &primTransformB,const Transform3 &relTransformB,int primIndexB,\
				float objsInContactDist);

typedef int (*PrimContacts)(
				ContactPoint *cp,
				const CollPrim & primA,const Transform3 &primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB,const Transform3 &primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist);

PRIMCONTACTFUNC(primContactsDummy				)

PRIMCONTACTFUNC(primContactsSphereSphere		)
PRIMCONTACTFUNC(primContactsSphereBox			)
PRIMCONTACTFUNC(primContactsSphereCapsule		)
PRIMCONTACTFUNC(primContactsBoxSphere			)
PRIMCONTACTFUNC(primContactsBoxBox				)
PRIMCONTACTFUNC(primContactsBoxCapsule			)
PRIMCONTACTFUNC(primContactsCapsuleSphere		)
PRIMCONTACTFUNC(primContactsCapsuleBox			)
PRIMCONTACTFUNC(primContactsCapsuleCapsule		)
PRIMCONTACTFUNC(primContactsShapeHeightField	)
PRIMCONTACTFUNC(primContactsHeightFieldShape	)
PRIMCONTACTFUNC(primContactsLargeMeshShape		)
PRIMCONTACTFUNC(primContactsShapeLargeMesh		)

PRIMCONTACTFUNC(primContactsSphereConvex		)
PRIMCONTACTFUNC(primContactsBoxConvex			)
PRIMCONTACTFUNC(primContactsCapsuleConvex		)
PRIMCONTACTFUNC(primContactsHeightFieldConvex	)
PRIMCONTACTFUNC(primContactsConvexConvex		)
PRIMCONTACTFUNC(primContactsConvexSphere		)
PRIMCONTACTFUNC(primContactsConvexBox			)
PRIMCONTACTFUNC(primContactsConvexCapsule		)
PRIMCONTACTFUNC(primContactsConvexHeightField	)

/*
	衝突判定関数テーブル
		SP	BX	CP	HF	CM	TM	LM
	SP	○	○	○	○	○	×	○
	BX	○	○	○	○	○	×	○
	CP	○	○	○	○	○	×	○
	HF	○	○	○	×	○	×	×
	CM	○	○	○	○	○	×	○
	TM	×	×	×	×	×	×	×
	LM	○	○	○	×	○	×	×
 */

PrimContacts funcTbl_primContacts[PRIM_COUNT][PRIM_COUNT] = {
	{primContactsSphereSphere		,primContactsSphereBox			,primContactsSphereCapsule		,primContactsShapeHeightField		,primContactsSphereConvex		,primContactsDummy,	primContactsShapeLargeMesh},
	{primContactsBoxSphere			,primContactsBoxBox				,primContactsBoxCapsule			,primContactsShapeHeightField		,primContactsBoxConvex			,primContactsDummy,	primContactsShapeLargeMesh},
	{primContactsCapsuleSphere		,primContactsCapsuleBox			,primContactsCapsuleCapsule		,primContactsShapeHeightField		,primContactsCapsuleConvex		,primContactsDummy,	primContactsShapeLargeMesh},
	{primContactsHeightFieldShape	,primContactsHeightFieldShape	,primContactsHeightFieldShape	,primContactsDummy					,primContactsHeightFieldConvex	,primContactsDummy,	primContactsDummy},
	{primContactsConvexSphere		,primContactsConvexBox			,primContactsConvexCapsule		,primContactsConvexHeightField		,primContactsConvexConvex		,primContactsDummy,	primContactsShapeLargeMesh},
	{primContactsDummy				,primContactsDummy				,primContactsDummy				,primContactsDummy					,primContactsDummy				,primContactsDummy,	primContactsDummy},
	{primContactsLargeMeshShape		,primContactsLargeMeshShape		,primContactsLargeMeshShape		,primContactsDummy					,primContactsLargeMeshShape		,primContactsDummy,	primContactsDummy},
};

///////////////////////////////////////////////////////////////////////////////
// ContactPair Function

#define SET_CONTACT_POINT(contactPoint,dist,nml,pntA,trnsA,primA,pntB,trnsB,primB) \
	contactPoint.distance = dist;\
	contactPoint.setNormal(nml);\
	contactPoint.setA(pntA.localPoint, trnsA, primA);\
	contactPoint.setB(pntB.localPoint, trnsB, primB);\
	contactPoint.subData.type = SubData::SubDataNone;

int primContactsDummy(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{
	(void) cp;
	(void) primA;
	(void) primTransformA;
	(void) relTransformA;
	(void) primIndexA;
	(void) primB;
	(void) primTransformB;
	(void) relTransformB;
	(void) primIndexB;
	(void) objsInContactDist;

	return 0;
}

int primContactsBoxBox(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	BoxPoint boxPointA;
	BoxPoint boxPointB;
	float distance = boxBoxDistance( testNormal, boxPointA, boxPointB, 
								   primA.getBox(), primTransformA,
								   primB.getBox(), primTransformB, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,-testNormal,
			boxPointA,relTransformA,primIndexA,
			boxPointB,relTransformB,primIndexB);
		return 1;
	}

	return 0;
}

int primContactsBoxCapsule(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	BoxPoint boxPointA;
	CapsulePoint capsulePointB;
	float distance = boxCapsuleDistance( testNormal, boxPointA, capsulePointB,
								   primA.getBox(), primTransformA,
								   primB.getCapsule(), primTransformB, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,-testNormal,
			boxPointA, relTransformA, primIndexA,
			capsulePointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

int primContactsBoxSphere(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	BoxPoint boxPointA;
	SpherePoint spherePointB;
	float distance = boxSphereDistance( testNormal, boxPointA, spherePointB,
								  primA.getBox(), primTransformA,
								  primB.getSphere(), primTransformB, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,-testNormal,
			boxPointA, relTransformA, primIndexA,
			spherePointB, relTransformB, primIndexB);
		return 1;
	}
	
	return 0;
}

int primContactsCapsuleBox(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	CapsulePoint capsulePointA;
	BoxPoint boxPointB;
	float distance = boxCapsuleDistance( testNormal, boxPointB, capsulePointA,
								   primB.getBox(), primTransformB,
								   primA.getCapsule(), primTransformA, objsInContactDist );
	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,testNormal,
			capsulePointA, relTransformA, primIndexA,
			boxPointB, relTransformB, primIndexB);
		return 1;
	}
	
	return 0;
}

int primContactsCapsuleCapsule(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	CapsulePoint capsulePointA;
	CapsulePoint capsulePointB;
	float distance = capsuleCapsuleDistance( testNormal, capsulePointA, capsulePointB,
									   primA.getCapsule(), primTransformA,
									   primB.getCapsule(), primTransformB, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,-testNormal,
			capsulePointA, relTransformA, primIndexA,
			capsulePointB, relTransformB, primIndexB);

		return 1;
	}

	return 0;
}

int primContactsCapsuleSphere(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	CapsulePoint capsulePointA;
	SpherePoint spherePointB;
	float distance = capsuleSphereDistance( testNormal, capsulePointA, spherePointB,
									  primA.getCapsule(), primTransformA,
									  primB.getSphere(), primTransformB, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,-testNormal,
			capsulePointA, relTransformA, primIndexA,
			spherePointB, relTransformB, primIndexB);
		
		return 1;
	}

	return 0;
}

int primContactsSphereBox(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	SpherePoint spherePointA;
	BoxPoint boxPointB;
	float distance = boxSphereDistance( testNormal, boxPointB, spherePointA,
								  primB.getBox(), primTransformB,
								  primA.getSphere(), primTransformA, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,testNormal,
			spherePointA, relTransformA, primIndexA,
			boxPointB, relTransformB, primIndexB);

		return 1;
	}
	
	return 0;
}

int primContactsSphereCapsule(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal;
	SpherePoint spherePointA;
	CapsulePoint capsulePointB;
	float distance = capsuleSphereDistance( testNormal, capsulePointB, spherePointA,
									  primB.getCapsule(), primTransformB,
									  primA.getSphere(), primTransformA, objsInContactDist );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,testNormal,
			spherePointA, relTransformA, primIndexA,
			capsulePointB, relTransformB, primIndexB);

		return 1;
	}
	
	return 0;
}

int primContactsSphereSphere(
				ContactPoint *cp,
				const CollPrim & primA, const Transform3 & primTransformA,const Transform3 &relTransformA,int primIndexA,
				const CollPrim & primB, const Transform3 & primTransformB,const Transform3 &relTransformB,int primIndexB,
				float objsInContactDist)
{

	Vector3 testNormal(0.0f);
	SpherePoint spherePointA;
	SpherePoint spherePointB;
	float distance = sphereSphereDistance( testNormal, spherePointA, spherePointB,
									 primA.getSphere(), primTransformA,
									 primB.getSphere(), primTransformB );

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0],distance,-testNormal,
			spherePointA, relTransformA, primIndexA,
			spherePointB, relTransformB, primIndexB);

		return 1;
	}
	
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Contact Filter

inline bool isCollidablePrims(const CollPrim &primA,const CollPrim &primB)
{
	return (primA.getContactFilterSelf()&primB.getContactFilterTarget()) && 
			(primA.getContactFilterTarget()&primB.getContactFilterSelf());
}

///////////////////////////////////////////////////////////////////////////////
// closestContact

bool
closestContact(ContactPair &contactPair,
				const CollObject & objA, const Transform3 & transformA,
				const CollObject & objB, const Transform3 & transformB,
				float objsInContactDist)
{
	contactPair.numContacts = 0;

	{
		PrimIterator itrPrimA(objA);
		for(int i=0;i<objA.getNumPrims();i++,++itrPrimA) {
			const CollPrim &primA = *itrPrimA;
			Transform3 relTransformA = primA.getObjectRelTransform();
			Transform3 primTransformA = transformA * relTransformA;

			PrimIterator itrPrimB(objB);
			for(int j=0;j<objB.getNumPrims();j++,++itrPrimB) {
				const CollPrim &primB = *itrPrimB;

				if(isCollidablePrims(primA,primB)) {
					Transform3 relTransformB = primB.getObjectRelTransform();
					Transform3 primTransformB = transformB * relTransformB;
					
					ContactPoint cp[NUMCONTACTS_PER_BODIES];
					int newContacts = funcTbl_primContacts[primA.getType()][primB.getType()](
						cp,
						primA,primTransformA,relTransformA,i,
						primB,primTransformB,relTransformB,j,
						objsInContactDist);
					
					for(int k=0;k<newContacts;k++) {
						int replaceId = findNearestContactPoint(contactPair.contactPoints,contactPair.numContacts,cp[k].getLocalPointA());
						
						if(replaceId < 0) {
							if(contactPair.numContacts < 4) {
								replaceId = contactPair.numContacts++;
							}
							else {
								replaceId = sort4ContactPoints(contactPair.contactPoints,cp[k].getLocalPointA(),cp[k].distance);
							}
						}
						
						contactPair.contactPoints[replaceId] = cp[k];
					}
				}
			}
		}
	}

	return (contactPair.numContacts > 0);
}

///////////////////////////////////////////////////////////////////////////////
// closestContactCCD

#define CCD_STACK_COUNT 10

struct CcdStackData
{
	float t0;
	float t1;
	int flag;

	CcdStackData(float t0_,float t1_,int f)
	{
		t0 = t0_;
		t1 = t1_;
		flag = f;
	}

	CcdStackData()
	{
		t0 = t1 = 0.0f;
		flag = 0;
	}
};

int testCcdPrim(ContactPair &contactPair,
			const CollObject &objA,const Transform3 &tA0,const Transform3 &tA1,
			const CollObject &objB,const Transform3 &tB0,const Transform3 &tB1,
			float objsInContactDist)
{
	CcdStackData lastToi;
	int loopCount = 0;

	Vector3 distA = tA1.getTranslation()-tA0.getTranslation();
	Vector3 distB = tB1.getTranslation()-tB0.getTranslation();
	Vector3 dir = distA - distB;
	float len = length(dir);
	float rA = objA.getCcdRadius();

	{
		ContactPoint cp[NUMCONTACTS_PER_BODIES];

		SimpleStack<CcdStackData> cs;

		cs.push(CcdStackData(0.0f,1.0f,0));

		int collisionCount = 0;

		bool collide = false;

		do {
			// スタックから取り出す
			CcdStackData sd = cs.pop();
			
			// バウンディング球を生成
			float mid = 0.5f * (sd.t0 + sd.t1);
			float r = (mid - sd.t0) * len + rA;

			CollPrim sphere;
			sphere.setSphere(Sphere(r));
			sphere.setObjectRelTransform(Transform3::identity());
			
			// バウンディング球とオブジェクトを衝突判定
			Transform3 trA = Transform3::translation(mid*dir) * tA0;
			collisionCount = 0;

			{
				PrimIterator itrPrim(objB);
				for(int j=0;j<objB.getNumPrims();j++,++itrPrim) {
					const CollPrim &primB = *itrPrim;
					Transform3 relTransformB = primB.getObjectRelTransform();
					Transform3 primTransformB = tB0 * relTransformB;
					
					collisionCount += funcTbl_primContacts[sphere.getType()][primB.getType()](
									cp,
									sphere,trA,Transform3::identity(),0,
									primB,primTransformB,relTransformB,j,
									objsInContactDist);
				}
			}

			// 衝突ならば、処理を続行
			if(collisionCount == 0)
				continue;

			loopCount++;

			// 衝突時の情報を保存
			if(loopCount > 1) {
				collide = true;
				lastToi = sd;
			}

			// 衝突しており、時間間隔が充分小さいのでループを抜ける
			if((sd.t1 - sd.t0) < CCD_THRESHOLD_MIN) {
				break;
			}
			
			// 前半部であれば、後半部は必要ないのでpop
			if(sd.flag == 1) {
				cs.pop();
			}

			// 衝突ならばスタックに追加
			if(cs.getStackCount() < CCD_STACK_COUNT-1) {
				cs.push(CcdStackData(mid,sd.t1,0)); // 後半部
				cs.push(CcdStackData(sd.t0,mid,1)); // 前半部
			}
		} while(!cs.isEmpty());

		if(!collide || (lastToi.t1 - lastToi.t0) > CCD_THRESHOLD_MAX) return 0;
	}

	float mid = 0.5f * (lastToi.t0 + lastToi.t1);

	// 形状で衝突判定
	{
		Transform3 trA = interpTransform(mid,tA0,tA1);
		closestContact(contactPair,objA,trA,objB,tB0,objsInContactDist);
	}
	
	// 形状で衝突していない場合、バウンディング球で衝突判定を行う
	if(contactPair.numContacts == 0) {
		ContactPoint cp[NUMCONTACTS_PER_BODIES];

		float r = (mid - lastToi.t0) * len + rA;

		CollPrim sphere;
		sphere.setSphere(Sphere(r));
		sphere.setObjectRelTransform(Transform3::identity());

		// 衝突判定
		Transform3 trA = Transform3::translation(mid*dir)*tA0;
		contactPair.numContacts = 0;

		PrimIterator itrPrim(objB);
		for(int j=0;j<objB.getNumPrims();j++,++itrPrim) {
			const CollPrim &primB = *itrPrim;
			
			Transform3 relTransformB = primB.getObjectRelTransform();
			Transform3 primTransformB = tB0 * relTransformB;
			
			int newContacts = funcTbl_primContacts[sphere.getType()][primB.getType()](
							cp,
							sphere,trA,Transform3::identity(),0,
							primB,primTransformB,relTransformB,j,
							objsInContactDist);

			for(int k=0;k<newContacts;k++) {
				int replaceId = findNearestContactPoint(contactPair.contactPoints,contactPair.numContacts,cp[k].getLocalPointA());

				if(replaceId < 0) {
					if(contactPair.numContacts < 4) {
						replaceId = contactPair.numContacts++;
					}
					else {
						replaceId = sort4ContactPoints(contactPair.contactPoints,cp[k].getLocalPointA(),cp[k].distance);
					}
				}
				
				contactPair.contactPoints[replaceId] = cp[k];
			}
		}
	}

	return contactPair.numContacts;
}

// A:Primitive (Dynamic)
// B:LargeMesh (Static)
int testCcdLargeMesh(ContactPair &contactPair,
			const CollObject &objA,const Transform3 &tA0,const Transform3 &tA1,
			const CollObject &objB,const Transform3 &tB0,const Transform3 &tB1,
			float objsInContactDist)
{
	(void) objsInContactDist;
	(void) tB1;
	
	contactPair.numContacts = 0;
	
	LargeTriMesh *largeMesh = objB.getDefPrim().getLargeMesh();
	
	Transform3 transformB,transformBA0,transformBA1;

	// Aローカル→Bローカルへの変換
	transformB = tB0 * objB.getDefPrim().getObjectRelTransform();
	transformBA0 = orthoInverse(transformB) * tA0;
	transformBA1 = orthoInverse(transformB) * tA1;

	Vector3 dir = transformBA1.getTranslation()-transformBA0.getTranslation();
	float len = length(dir);
	float rA = objA.getCcdRadius();
	
	
	VecInt3 aabbMinL = largeMesh->getLocalPosition(
							minPerElem(transformBA0.getTranslation(),transformBA1.getTranslation()) - Vector3(rA));
	VecInt3 aabbMaxL = largeMesh->getLocalPosition(
							maxPerElem(transformBA0.getTranslation(),transformBA1.getTranslation()) + Vector3(rA));
	
	{

		for(uint32_t i=0;i<largeMesh->numIslands;i++) {
			// AABBチェック
			PfxAABB16 aabbB = largeMesh->aabbList[i];
			if(aabbMaxL.getX() < XMin(aabbB) || aabbMinL.getX() > XMax(aabbB)) continue;
			if(aabbMaxL.getY() < YMin(aabbB) || aabbMinL.getY() > YMax(aabbB)) continue;
			if(aabbMaxL.getZ() < ZMin(aabbB) || aabbMinL.getZ() > ZMax(aabbB)) continue;

			// islandに含まれるTriangleとのCCD判定
			TriMesh *island = &largeMesh->islands[i];
			
			for(uint32_t f=0;f<island->numFacets;f++) {
				const MeshFacet &facet = island->facets[f];
				
				Vector3 facetNormal = read_Vector3(facet.normal);

				Vector3 facetPnts[3] = {
					island->verts[facet.vertIndices[0]],
					island->verts[facet.vertIndices[1]],
					island->verts[facet.vertIndices[2]],
				};
				
				uint32_t edgeChk = 
					((island->edges[facet.edgeIndices[0]].angle==EDGE_CONVEX)?0x01:0x00) |
					((island->edges[facet.edgeIndices[1]].angle==EDGE_CONVEX)?0x02:0x00) |
					((island->edges[facet.edgeIndices[2]].angle==EDGE_CONVEX)?0x04:0x00);

				CcdStackData lastToi;
				int loopCount = 0;
				bool collide = false;			
		
				SimpleStack<CcdStackData> cs;
				
				cs.push(CcdStackData(0.0f,1.0f,0));
				
				do {
					// スタックから取り出す
					CcdStackData sd = cs.pop();
					
					// バウンディング球を生成
					float mid = 0.5f * (sd.t0 + sd.t1);
					float r = (mid - sd.t0) * len + rA;
					Vector3 posA = mid*dir+transformBA0.getTranslation();
					Vector3 pntOnA(0.0f);
					
					// バウンディング球-トライアングル衝突判定
					distancePointAndTriangle(facetPnts[0],facetPnts[1],facetPnts[2],posA,pntOnA);
					
					// 衝突ならば、処理を続行
					if(lengthSqr(pntOnA-posA) > r*r)
						continue;

					loopCount++;

					// 衝突時の情報を保存
					if(loopCount > 1) {
						collide = true;
						lastToi = sd;
					}

					// 衝突しており、時間間隔が充分小さいのでループを抜ける
					if((sd.t1 - sd.t0) < CCD_THRESHOLD_MIN) {
						break;
					}
					
					// 前半部であれば、後半部は必要ないのでpop
					if(sd.flag == 1) {
						cs.pop();
					}

					// 衝突ならばスタックに追加
					if(cs.getStackCount() < CCD_STACK_COUNT-1) {
						cs.push(CcdStackData(mid,sd.t1,0)); // 後半部
						cs.push(CcdStackData(sd.t0,mid,1)); // 前半部
					}
				} while(!cs.isEmpty());

				if(!collide || (lastToi.t1 - lastToi.t0) > CCD_THRESHOLD_MAX) continue;
				
				float mid = 0.5f * (lastToi.t0 + lastToi.t1);
				
				// 形状で衝突判定を行う
				// Todo:
				
				// 形状で衝突していない場合、バウンディング球で衝突判定を行う
				{
					float r = (mid - lastToi.t0) * len + rA;
					
					// 衝突判定
					Transform3 trA = Transform3::translation(mid*dir) * transformBA0;

					Vector3 pntOnA(0.0f);
					Vector3 pntOnB(0.0f);
					Vector3 sepAxis(0.0f);
					
					int n = triangleSphereContact(
								sepAxis,pntOnB,pntOnA,
								facetNormal,facetPnts[0],facetPnts[1],facetPnts[2],facet.thickness,edgeChk,
								r,trA.getTranslation());

					// 衝突点を追加
					if(n > 0) {
						ContactPoint cp;
						
						float s,t;
						get_ST(s,t,facetPnts[1]-facetPnts[0],facetPnts[2]-facetPnts[0],pntOnB-facetPnts[0]);
						SubData sub;
						sub.type = SubData::SubDataFacetLocal;
						sub.setIslandIndex(i);
						sub.setFacetIndex(f);
						sub.setFacetLocalS(s);
						sub.setFacetLocalT(t);
						
						// それぞれのローカル系に変換して格納
						cp.distance = -length(pntOnB - pntOnA);
						cp.setNormal(-(transformB.getUpper3x3()*sepAxis));
						Point3 pA(orthoInverse(trA) * Point3(pntOnA));
						Point3 pB(pntOnB);
						cp.setA(pA, Transform3::identity(), 0);
						cp.setB(pB, objB.getDefPrim().getObjectRelTransform(), 0);
						cp.subData = sub;
						
						int replaceId = findNearestContactPoint(contactPair.contactPoints,contactPair.numContacts,cp.getLocalPointA());
						
						if(replaceId < 0) {
							if(contactPair.numContacts < 4) {
								replaceId = contactPair.numContacts++;
							}
							else {
								replaceId = sort4ContactPoints(contactPair.contactPoints,cp.getLocalPointA(),cp.distance);
							}
						}
						
						contactPair.contactPoints[replaceId] = cp;
					}
				}
			}
		}
	}

	return contactPair.numContacts;
}

uint32_t findContactCCD(ContactPair &contactPair,
				const CollObject &objA,const Transform3 &tA0,const Transform3 &tA1,bool useCcdA,
				const CollObject &objB,const Transform3 &tB0,const Transform3 &tB1,bool useCcdB,
				float objsInContactDist)
{
	int count = 0;

	Vector3 distA = tA1.getTranslation()-tA0.getTranslation();
	Vector3 distB = tB1.getTranslation()-tB0.getTranslation();

	Vector3 relativeDistance = distA - distB;
	float d = length(relativeDistance);
	
	float checkDA = useCcdA?CCD_ENABLE_DISTANCE*objA.getCcdRadius():0.0f;
	float checkDB = useCcdB?CCD_ENABLE_DISTANCE*objB.getCcdRadius():0.0f;
	bool checkA = false,checkB = false;

	if(useCcdA && d > checkDA) {
		checkA = true;
	}
	if(useCcdB && d > checkDB && checkDB > checkDA) {
		checkA = false;
		checkB = true;
	}

	if(checkA) {
		if(objB.getDefPrim().getType() == LARGEMESH) {
			count = testCcdLargeMesh(contactPair,objA,tA0,tA1,objB,tB0,tB1,objsInContactDist);
		}
		else {
			count = testCcdPrim(contactPair,objA,tA0,tA1,objB,tB0,tB1,objsInContactDist);
		}
	}
	else if(checkB) {
		if(objA.getDefPrim().getType() == LARGEMESH) {
			count = testCcdLargeMesh(contactPair,objB,tB0,tB1,objA,tA0,tA1,objsInContactDist);
		}
		else {
			count = testCcdPrim(contactPair,objB,tB0,tB1,objA,tA0,tA1,objsInContactDist);
		}
		for(int i=0;i<contactPair.numContacts;i++)
			contactPair.contactPoints[i].exchange();
	}
	else {
		// 通常の判定
		return closestContact(contactPair,objA,tA0,objB,tB0,objsInContactDist);
	}
	
	(void) count;
	
	return (contactPair.numContacts > 0);
}

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
#include "Physics/Base/PerfCounter.h"
#include "Physics/Base/SimdFunc.h"

#include <float.h>
#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Box.h"
#include "Capsule.h"
#include "TriMesh.h"
#include "intersectFunction.h"
#include "PfxContactCache.h"
#include "sat_mesh_utils.h"
#include "SubData.h"

// 分離軸が見つかった場合はすぐに処理を抜けるため最短距離が返されるわけではないことに注意

/*
	○ カプセル分離軸(x19)
	面法線
	カプセル軸 x エッジ0
	カプセル軸 x エッジ1
	カプセル軸 x エッジ2
	((カプセル点0-面点0) x エッジ0) x エッジ0
	((カプセル点0-面点1) x エッジ1) x エッジ1
	((カプセル点0-面点2) x エッジ2) x エッジ2
	((カプセル点1-面点0) x エッジ0) x エッジ0
	((カプセル点1-面点1) x エッジ1) x エッジ1
	((カプセル点1-面点2) x エッジ2) x エッジ2
	((面点0-カプセル点0) x カプセル軸) x カプセル軸
	((面点1-カプセル点0) x カプセル軸) x カプセル軸
	((面点2-カプセル点0) x カプセル軸) x カプセル軸
	面点0-カプセル点0
	面点1-カプセル点0
	面点2-カプセル点0
	面点0-カプセル点1
	面点1-カプセル点1
	面点2-カプセル点1
 */

static inline bool checkSAT(const Vector3 &axis,float AMin,float AMax,float BMin,float BMax,float &distMin,Vector3 &axisMin)
{
	// ■ 非接触.

	// A:          +----+
	// B:+----+
	if(BMax <= AMin) {
		return true;
	}

	// A:+----+
	// B:          +----+
	else if(AMax <= BMin) { // Todo:SIMD
		return true;
	}
	// ■ 内包

	//A:      +--+
	//B:   +------+
	if(BMin < AMin && AMax < BMax) {
		float d = AMin-BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	}

	//A:   +------+
	//B:    +--+
	else if(AMin < BMin && BMax < AMax) {
		float d = AMin-BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	}

	// ■ 接触
	
	// A:   +----+
	// B:+----+
	else if(BMin < AMin && AMin < BMax) {
		float d = AMin-BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	}

	// A:   +----+
	// B:      +----+
	else if(AMin < BMin && BMin < AMax) {
		float d = BMin-AMax;
		if(distMin < d) {
			distMin = d;
			axisMin = -axis;
		}
	}

	return false;
}

bool triangleCapsuleContact(Vector3 &axis,Vector3 &pointsOnTriangle,Vector3 &pointsOnCapsule,
							const Vector3 &normal,const Vector3 &p0,const Vector3 &p1,const Vector3 &p2,const float thickness,uint32_t edgeChk,
							float capsuleRadius,const Vector3 &capP0,const Vector3 &capP1)
{
	// 最も浅い貫通深度とそのときの分離軸
	float distMin = -FLT_MAX;
	Vector3 axisMin(0.0f);

	//-------------------------------------------
	// １．分離軸判定
	{
		const Vector3 facetPnts[6] = {
			p0,p1,p2,p0-thickness*normal,p1-thickness*normal,p2-thickness*normal
		};

		const Vector3 capPnts[2] = {
			capP0,capP1
		};

		{
			const Vector3 &sepAxis = normal;

			// 分離平面
			Plane plane(sepAxis,facetPnts[0]);

			// Capsule(B)を分離軸に投影して範囲を取得
			float test1,test2,BMin,BMax;
			test1 = plane.onPlane(capP0);
			test2 = plane.onPlane(capP1);
			BMax = (test1<test2?test2:test1) + capsuleRadius;
			BMin = (test1<test2?test1:test2) - capsuleRadius;

			// 判定
			if(BMin > 0.0f || BMax < -thickness) {
				return false;
			}

			if(distMin < BMin) {
				distMin = BMin;
				axisMin = -sepAxis;
			}
		}

		//-------------------------------------------
		// カプセル軸 x 面エッジ0,1,2
		// ※Triangles座標系 (Aローカル)

		{
			for(int e=0;e<3;e++) {
				Vector3 sepAxis = cross(capP1-capP0,facetPnts[(e+1)%3]-facetPnts[e]);
				float l=length(sepAxis);
				if(l < 0.00001f) continue;
				sepAxis /= l;
				
				if(dot(normal,sepAxis) > 0.0f)
					sepAxis = -sepAxis;
				
				// Triangleを分離軸に投影
				float AMin=FLT_MAX,AMax=-FLT_MAX;
				getProjAxisPnts6(facetPnts,sepAxis,AMin,AMax);
				
				// カプセルを分離軸に投影
				float BMin=FLT_MAX,BMax=-FLT_MAX;
				getProjAxisPnts2(capPnts,sepAxis,BMin,BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;
				
				if(checkSAT(sepAxis,AMin,AMax,BMin,BMax,distMin,axisMin)) {
					return false;
				}
			}
		}

		//-------------------------------------------
		// ((カプセル点0-面点0,1,2) x エッジ0,1,2) x エッジ0,1,2
		// ※Triangles座標系 (Aローカル)
		
		{
			for(int e=0;e<3;e++) {
				Vector3 edge = facetPnts[(e+1)%3]-facetPnts[e];
				Vector3 sepAxis = normalize(cross(cross(capP0-facetPnts[e],edge),edge));
				
				if(dot(normal,sepAxis) > 0.0f)
					sepAxis = -sepAxis;
				
				// Triangleを分離軸に投影
				float AMin=FLT_MAX,AMax=-FLT_MAX;
				getProjAxisPnts6(facetPnts,sepAxis,AMin,AMax);
				
				// カプセルを分離軸に投影
				float BMin=FLT_MAX,BMax=-FLT_MAX;
				getProjAxisPnts2(capPnts,sepAxis,BMin,BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;
				
				if(checkSAT(sepAxis,AMin,AMax,BMin,BMax,distMin,axisMin)) {
					return false;
				}
			}
		}

		//-------------------------------------------
		// ((カプセル点1-面点0,1,2) x エッジ0,1,2) x エッジ0,1,2
		// ※Triangles座標系 (Aローカル)
		
		{
			for(int e=0;e<3;e++) {
				Vector3 edge = facetPnts[(e+1)%3]-facetPnts[e];
				Vector3 sepAxis = normalize(cross(cross(capP1-facetPnts[e],edge),edge));
				
				if(dot(normal,sepAxis) > 0.0f)
					sepAxis = -sepAxis;
				
				// Triangleを分離軸に投影
				float AMin=FLT_MAX,AMax=-FLT_MAX;
				getProjAxisPnts6(facetPnts,sepAxis,AMin,AMax);
				
				// カプセルを分離軸に投影
				float BMin=FLT_MAX,BMax=-FLT_MAX;
				getProjAxisPnts2(capPnts,sepAxis,BMin,BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;
				
				if(checkSAT(sepAxis,AMin,AMax,BMin,BMax,distMin,axisMin)) {
					return false;
				}
			}
		}

		//-------------------------------------------
		// ((面点0,1,2-カプセル点0) x カプセル軸) x カプセル軸
		// ※Triangles座標系 (Aローカル)

		{
			for(int e=0;e<3;e++) {
				Vector3 capdir = capP1-capP0;
				Vector3 sepAxis = cross(cross(facetPnts[e]-capP0,capdir),capdir);
				float l=length(sepAxis);
				if(l < 0.00001f) continue;
				sepAxis /= l;

				if(dot(normal,sepAxis) > 0.0f)
					sepAxis = -sepAxis;

				// Triangleを分離軸に投影
				float AMin=FLT_MAX,AMax=-FLT_MAX;
				getProjAxisPnts6(facetPnts,sepAxis,AMin,AMax);
				
				// カプセルを分離軸に投影
				float BMin=FLT_MAX,BMax=-FLT_MAX;
				getProjAxisPnts2(capPnts,sepAxis,BMin,BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;
				
				if(checkSAT(sepAxis,AMin,AMax,BMin,BMax,distMin,axisMin)) {
					return false;
				}
			}
		}

		//-------------------------------------------
		// 面点0,1,2-カプセル点0
		// ※Triangles座標系 (Aローカル)

		{
			for(int e=0;e<3;e++) {
				Vector3 sepAxis = normalize(facetPnts[e]-capP0);

				// Triangleを分離軸に投影
				float AMin=FLT_MAX,AMax=-FLT_MAX;
				getProjAxisPnts6(facetPnts,sepAxis,AMin,AMax);
				
				// カプセルを分離軸に投影
				float BMin=FLT_MAX,BMax=-FLT_MAX;
				getProjAxisPnts2(capPnts,sepAxis,BMin,BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;
				
				if(checkSAT(sepAxis,AMin,AMax,BMin,BMax,distMin,axisMin)) {
					return false;
				}
			}
		}

		//-------------------------------------------
		// 面点0,1,2-カプセル点1
		// ※Triangles座標系 (Aローカル)

		{
			for(int e=0;e<3;e++) {
				Vector3 sepAxis = normalize(facetPnts[e]-capP1);
				
				// Triangleを分離軸に投影
				float AMin=FLT_MAX,AMax=-FLT_MAX;
				getProjAxisPnts6(facetPnts,sepAxis,AMin,AMax);
				
				// カプセルを分離軸に投影
				float BMin=FLT_MAX,BMax=-FLT_MAX;
				getProjAxisPnts2(capPnts,sepAxis,BMin,BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;
				
				if(checkSAT(sepAxis,AMin,AMax,BMin,BMax,distMin,axisMin)) {
					return false;
				}
			}
		}

		// 面に厚みがある場合の補助的な判定（面法線 x カプセル軸）
		// 交差するかしないかだけを判定
		if(thickness > 0.0f) {
			{
				Vector3 sepAxis = cross(capP1-capP0,normal);

				// Triangleを分離軸に投影
				float AMin=FLT_MAX,AMax=-FLT_MAX;
				getProjAxisPnts6(facetPnts,sepAxis,AMin,AMax);
				
				// カプセルを分離軸に投影
				float BMin=FLT_MAX,BMax=-FLT_MAX;
				getProjAxisPnts2(capPnts,sepAxis,BMin,BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;

				if(BMax <= AMin || AMax <= BMin) {
					return false;
				}
			}
			for(int e=0;e<3;e++) {
				Vector3 edge = facetPnts[(e+1)%3]-facetPnts[e];
				Vector3 sepAxis = cross(normal,edge);
				
				// Triangleを分離軸に投影
				float AMin=FLT_MAX,AMax=-FLT_MAX;
				getProjAxisPnts6(facetPnts,sepAxis,AMin,AMax);
				
				// カプセルを分離軸に投影
				float BMin=FLT_MAX,BMax=-FLT_MAX;
				getProjAxisPnts2(capPnts,sepAxis,BMin,BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;

				if(BMax <= AMin || AMax <= BMin) {
					return false;
				}
			}
		}
	}
	
	axis = axisMin;
	
	//-------------------------------------------
	// ２．衝突点の探索
	
	{
		// 分離軸方向に引き離す(最近接を判定するため、交差回避させる)
		Vector3 sepAxis = 1.1f * fabsf(distMin) * axisMin;
		
		const Vector3 facetPnts[3] = {
			p0 + sepAxis,
			p1 + sepAxis,
			p2 + sepAxis,
		};

		//--------------------------------------------------------------------
		// 衝突点の探索

		float depthSqrMin = FLT_MAX;
		Vector3 sA,sB;

		//--------------------------------------------------------------------
		//Triangleの頂点 -> Capsule

		float distSqr;

		// カプセルの線分と面のエッジx3の最近接点の算出
		distanceTwoLines(capP0,capP1,facetPnts[0],facetPnts[1],sB,sA);
		distSqr = lengthSqr(sA-sB);
		if(distSqr < depthSqrMin) {
			depthSqrMin = distSqr;
			pointsOnTriangle = sA;
			pointsOnCapsule = sB + normalize(sA-sB)*capsuleRadius;
		}
		
		distanceTwoLines(capP0,capP1,facetPnts[1],facetPnts[2],sB,sA);
		distSqr = lengthSqr(sA-sB);
		if(distSqr < depthSqrMin) {
			depthSqrMin = distSqr;
			pointsOnTriangle = sA;
			pointsOnCapsule = sB + normalize(sA-sB)*capsuleRadius;
		}

		distanceTwoLines(capP0,capP1,facetPnts[2],facetPnts[0],sB,sA);
		distSqr = lengthSqr(sA-sB);
		if(distSqr < depthSqrMin) {
			depthSqrMin = distSqr;
			pointsOnTriangle = sA;
			pointsOnCapsule = sB + normalize(sA - sB)*capsuleRadius;
		}

		// カプセルの端点と面の最近接点の算出
		distancePointAndTriangle(facetPnts[0],facetPnts[1],facetPnts[2],capP0,sA);
		distSqr = lengthSqr(sA-capP0);
		if(distSqr < depthSqrMin) {
			depthSqrMin = distSqr;
			pointsOnTriangle = sA;
			pointsOnCapsule = capP0 + normalize(sA-capP0)*capsuleRadius;
		}

		distancePointAndTriangle(facetPnts[0],facetPnts[1],facetPnts[2],capP1,sA);
		distSqr = lengthSqr(sA-capP1);
		if(distSqr < depthSqrMin) {
			depthSqrMin = distSqr;
			pointsOnTriangle = sA;
			pointsOnCapsule = capP1 + normalize(sA-capP1)*capsuleRadius;
		}
	
		pointsOnTriangle -= sepAxis;
	}

	// 面上の最近接点が凸エッジ上でない場合は法線を変える
	if( ((edgeChk&0x01)&&pointOnLine(pointsOnTriangle,p0,p1)) ||
		((edgeChk&0x02)&&pointOnLine(pointsOnTriangle,p1,p2)) ||
		((edgeChk&0x04)&&pointOnLine(pointsOnTriangle,p2,p0)) ) {
		axis=-normal;
	}

	return true;
}

int trianglesCapsuleContacts( Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, float *distance,
								   const TriMesh *meshA, const Transform3 & transformA,
								   Capsule capB, const Transform3 & transformB,float distanceThreshold)
{
	(void) distanceThreshold;

#ifdef ENABLE_PERF
	PerfCounter pc;
#endif

	Transform3 transformAB, transformBA;
	Matrix3 matrixBA;
	Vector3 offsetBA;

	// Bローカル→Aローカルへの変換.
	transformAB = orthoInverse(transformA) * transformB;

	// Aローカル→Bローカルへの変換.
	transformBA = orthoInverse(transformAB);

	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	//-------------------------------------------
	// 判定する面を絞り込む.

	uint8_t selFacets[NUMMESHFACETS] = { 0 };
	uint8_t numSelFacets = 0;

	// ※capsuleB座標系
	Vector3 aabbB = capB.GetAABB(Vector3(1,0,0));
	gatherFacets(meshA,(float*)&aabbB,offsetBA,matrixBA,selFacets,numSelFacets);

	if(numSelFacets == 0) {
		return 0;
	}
	
	//-------------------------------------------
	// 分離軸判定(SAT)
	// ※CapsuleB座標系 (Bローカル)で判定

	Vector3 vCapAB[2] = {
		Vector3(-capB.hLength, 0.0f, 0.0f),
		Vector3( capB.hLength, 0.0f, 0.0f)
	};
	
	PfxContactCache contacts;
	
	for(uint32_t f = 0; f < numSelFacets; f++) {
		const MeshFacet &facet = meshA->facets[selFacets[f]];
		
		Vector3 facetNormal = matrixBA * read_Vector3(facet.normal);

		Vector3 facetPntsA[3] = {
			offsetBA + matrixBA * meshA->verts[facet.vertIndices[0]],
			offsetBA + matrixBA * meshA->verts[facet.vertIndices[1]],
			offsetBA + matrixBA * meshA->verts[facet.vertIndices[2]],
		};
		
		Vector3 axis,pointsOnTriangle,pointsOnCapsule;

		uint32_t edgeChk = 
			((meshA->edges[facet.edgeIndices[0]].angle==EDGE_CONVEX)?0x00:0x01) |
			((meshA->edges[facet.edgeIndices[1]].angle==EDGE_CONVEX)?0x00:0x02) |
			((meshA->edges[facet.edgeIndices[2]].angle==EDGE_CONVEX)?0x00:0x04);
		
		if(triangleCapsuleContact(axis,pointsOnTriangle,pointsOnCapsule,
							facetNormal,facetPntsA[0],facetPntsA[1],facetPntsA[2],facet.thickness,edgeChk,
							capB.radius,vCapAB[0],vCapAB[1])) {
			// 衝突点を追加する
			contacts.add(
				-length(pointsOnCapsule - pointsOnTriangle),
				axis,
				pointsOnTriangle,
				pointsOnCapsule,
				selFacets[f]);
		}
	}
	
	for(uint32_t i=0;i<contacts.getNumContacts();i++) {
		normal[i] = transformB.getUpper3x3() * contacts.getNormal(i);
		pointA[i] = transformAB * Point3(contacts.getPointA(i));
		pointB[i] = Point3(contacts.getPointB(i));
		distance[i] = contacts.getDistance(i);

		const MeshFacet &facet = meshA->facets[contacts.getInfo(i)];
		Vector3 facetPnts[3] = {
			meshA->verts[facet.vertIndices[0]],	
			meshA->verts[facet.vertIndices[1]],	
			meshA->verts[facet.vertIndices[2]],	
		};

		float s,t;
		get_ST(s,t,facetPnts[1]-facetPnts[0],facetPnts[2]-facetPnts[0],Vector3(pointA[i])-facetPnts[0]);
		subData[i].type = SubData::SubDataFacetLocal;
		subData[i].setFacetIndex((uint8_t)contacts.getInfo(i));
		subData[i].setFacetLocalS(s);
		subData[i].setFacetLocalT(t);
	}
	
	return contacts.getNumContacts();
}

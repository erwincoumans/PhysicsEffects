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

#include <float.h>
#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/SimdFunc.h"

#include "TriMesh.h"
#include "intersectFunction.h"
#include "sat_mesh_utils.h"

#if 1   // USE GJK for Convex-Convex collision

#include "PfxGJKSolver.h"
#include "PfxGJKSupportFunc.h"

float closestConvexConvex( Vector3& normal, Point3 &pointA, Point3 &pointB,
						const ConvexMesh *meshA, const Transform3 & transformA,
						const ConvexMesh *meshB, const Transform3 & transformB,
						float distanceThreshold)
{
	(void) distanceThreshold;

	PfxGJKSolver gjk((void*)meshA,(void*)meshB,getSupportVertexConvex,getSupportVertexConvex);

	return gjk.collide(normal,pointA,pointB,transformA,transformB,FLT_MAX);
}

#else   // USE SAT for Convex-Convex collision

//#define ENABLE_PERF

static
bool closestVertAndFacet(
	const Vector3 &sepAxis,													// 分離軸
	const Vector3 *vertsA,const uint8_t *selVertsA,uint8_t numSelVertsA,	// Aの頂点情報
	const Vector3 *vertsB,const MeshFacet *facetsB,const uint8_t *selFacetsB,uint8_t numSelFacetsB,	// Bの面情報
	Vector3 &sA,Vector3 &sB,// 出力：最近接点
	float &distanceMin		// 出力：最近接距離
	)
{
	bool check = false;
	float depthMin = FLT_MAX;
	Vector3 minA(0.0f),minB(0.0f);

	for(uint8_t v=0;v<numSelVertsA;v++) {
		Vector3 vert = vertsA[selVertsA[v]];
		
		for(uint8_t f=0;f<numSelFacetsB;f++) {
			const MeshFacet &facet = facetsB[selFacetsB[f]];
			
			// 離れる方向と逆向きの面は排除
			if(dot(sepAxis,facet.normal) < 0.0f) continue;

			Vector3 facetPnts[3] = {
				vertsB[facet.vertIndices[0]],
				vertsB[facet.vertIndices[1]],
				vertsB[facet.vertIndices[2]],
			};
			
			Vector3 closest;
			distancePointAndTriangle(facetPnts[0],facetPnts[1],facetPnts[2],vert,closest);
			
			float d = lengthSqr(closest-vert);
			
			if(d < depthMin) {
				depthMin = d;
				minA = vert;
				minB = closest;
				check = true;
			}
		}
	}
	
	sA = minA;
	sB = minB;
	distanceMin = depthMin;
	
	return check;
}

static
bool closestTwoEdges(
	const Vector3 *vertsA,const MeshEdge *edgesA,const uint8_t *selEdgesA,uint8_t numSelEdgesA,	// Aのエッジ
	const Vector3 *vertsB,const MeshEdge *edgesB,const uint8_t *selEdgesB,uint8_t numSelEdgesB,	// Bのエッジ
	Vector3 &sA,Vector3 &sB,	// 出力：最近接点
	float &distanceMin			// 出力：最近接距離
	)
{
	bool check = false;
	float depthMin = FLT_MAX;
	Vector3 minA(0.0f),minB(0.0f);

	for(uint8_t eA=0;eA<numSelEdgesA;eA++) {
		const MeshEdge &edgeA = edgesA[selEdgesA[eA]];
		
		for(uint8_t eB=0;eB<numSelEdgesB;eB++) {
			const MeshEdge &edgeB = edgesB[selEdgesB[eB]];
			
			Vector3 s1,s2;
			distanceTwoLines(
				vertsA[edgeA.vertIndex[0]],vertsA[edgeA.vertIndex[1]],
				vertsB[edgeB.vertIndex[0]],vertsB[edgeB.vertIndex[1]],
				s1,s2);
			
			float d = lengthSqr(s1-s2);

			if(d < depthMin) {
				minA = s1;
				minB = s2;
				depthMin = d;
				check = true;
			}
		}
	}
	
	sA = minA;
	sB = minB;
	distanceMin = depthMin;
	
	return check;
}

// 分離軸が見つかった場合はすぐに抜けるため最短距離が返されるわけではないことに注意

float closestConvexConvex( Vector3& normal, Point3 &pointA, Point3 &pointB,
						const TriMesh *meshA, const Transform3 & transformA,
						const TriMesh *meshB, const Transform3 & transformB,
						float distanceThreshold )
{
	enum {A_AXIS=0, B_AXIS, CROSS_AXIS};
	uint8_t SATtype;
	uint8_t minFacetA=0,minFacetB=0;

#ifdef ENABLE_PERF
	PerfCounter pc;
#endif

	Transform3 transformAB, transformBA;
	Matrix3 matrixAB, matrixBA;
	Vector3 offsetAB, offsetBA;
	Vector3 sepW,sepA,sepB;

	// Bローカル→Aローカルへの変換
	transformAB = orthoInverse(transformA) * transformB;

	// Aローカル→Bローカルへの変換
	transformBA = orthoInverse(transformAB);

	matrixAB = transformAB.getUpper3x3();
	offsetAB = transformAB.getTranslation();
	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	// Aを引き離す方向
	sepW = normalize(transformA.getTranslation() - transformB.getTranslation());
	sepA = transpose(transformA.getUpper3x3()) * sepW;
    sepB = transpose(transformB.getUpper3x3()) * sepW;

	//-------------------------------------------
	// 判定する面を絞り込む

	uint8_t selFacetsA[NUMMESHFACETS] = {0};
	uint8_t selVertsA[NUMMESHVERTICES] = {0};
	uint8_t selEdgesA[NUMMESHEDGES] = {0};
	uint8_t numSelFacetsA = 0;
	uint8_t numSelVertsA = 0;
	uint8_t numSelEdgesA = 0;

	uint8_t selFacetsB[NUMMESHFACETS] = {0};
	uint8_t selVertsB[NUMMESHVERTICES] = {0};
	uint8_t selEdgesB[NUMMESHEDGES] = {0};
	uint8_t numSelFacetsB = 0;
	uint8_t numSelVertsB = 0;
	uint8_t numSelEdgesB = 0;

	Vector3 vertsBA[NUMMESHVERTICES]; // BローカルのA座標
	Vector3 vertsAB[NUMMESHVERTICES]; // AローカルのB座標

#ifdef ENABLE_PERF
	pc.countBegin("CC:Convert to local");
#endif

	// 座標をそれぞれのローカル系に変換する
	for(uint8_t v=0;v<meshA->numVerts;v++) {
		vertsBA[v] = offsetBA + matrixBA * meshA->verts[v];
	}

	for(uint8_t v=0;v<meshB->numVerts;v++) {
		vertsAB[v] = offsetAB + matrixAB * meshB->verts[v];
	}

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CC:Cull faces");
#endif

	// ※ConvexB座標系
	gatherFacets(meshA,meshB->half,offsetBA,matrixBA,selFacetsA,numSelFacetsA,selVertsA,numSelVertsA);

	// ※ConvexA座標系
	gatherFacets(meshB,meshA->half,offsetAB,matrixAB,selFacetsB,numSelFacetsB,selVertsB,numSelVertsB);

	if(numSelFacetsA == 0 || numSelFacetsB == 0) {
		return distanceThreshold;
	}

#ifdef ENABLE_PERF
	pc.countEnd();
#endif
	
	//-------------------------------------------
	// 分離軸判定(SAT)

#ifdef ENABLE_PERF
	pc.countBegin("CC:SAT facet normal");
#endif

	// 最も浅い貫通深度とそのときの分離軸
	float distMinA = -FLT_MAX;
	float distMinB = -FLT_MAX;
	float distMin = -FLT_MAX;
	Vector3 axisMinA(0.0f);
	Vector3 axisMinB(0.0f);
	Vector3 axisMin(0.0f);

	// ConvexAの面 -> ConvexB (Aローカル)
	// ※ConvexA座標系

	for(uint8_t f=0;f<numSelFacetsA;f++) {
		const MeshFacet &facet = meshA->facets[selFacetsA[f]];
		
		// 分離軸
		Vector3 axis = facet.normal;

		// 分離平面
		Plane planeA(axis,meshA->verts[facet.vertIndices[0]]);

		float convexBMin=FLT_MAX;

		// ConvexBを分離軸に投影して範囲を取得
		getProjPlane(vertsAB,selVertsB,numSelVertsB,planeA,convexBMin);
		
		// 判定
		if(convexBMin > 0.0f) {
			return convexBMin;
		}

		if(distMinA < convexBMin) {
			distMinA = convexBMin;
			axisMinA = -axis;
			minFacetA = selFacetsA[f];
		}
	}

	// ConvexBの面 -> ConvexA (Bローカル)
	// ※ConvexB座標系
	for(uint8_t f=0;f<numSelFacetsB;f++) {
		const MeshFacet &facet = meshB->facets[selFacetsB[f]];

		// 分離軸
		Vector3 axis = facet.normal;

		// 分離平面
		Plane planeB(axis,meshB->verts[facet.vertIndices[0]]);

		float convexAMin=FLT_MAX;
		
		// ConvexAを分離軸に投影して範囲を取得
		getProjPlane(vertsBA,selVertsA,numSelVertsA,planeB,convexAMin);
		
		// 判定
		if(convexAMin > 0.0f) {
			return convexAMin;
		}
		
		if(distMinB < convexAMin) {
			distMinB = convexAMin;
			axisMinB = axis;
			minFacetB = selFacetsB[f];
		}
	}

	distMin = distMinA<distMinB?distMinB:distMinA;
	axisMin = distMinA<distMinB?axisMinB:axisMinA;
	SATtype = distMinA<distMinB?B_AXIS:A_AXIS;

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CC:SAT Edge x Edge");
#endif

	///////////////////////////////////////////////////////////////////////////////
	// エッジ ConvexAの全エッジ×ConvexBの全エッジ
	// ※ConvexB座標系

	// エッジを集める
	gatherEdges2(meshA,selFacetsA,numSelFacetsA,selEdgesA,numSelEdgesA);
	gatherEdges2(meshB,selFacetsB,numSelFacetsB,selEdgesB,numSelEdgesB);

	// 同じ貫通深度であれば面判定のほうを優先させる
	const float depthRatio = 1.01f;

	uint8_t edgeIdA=0,edgeIdB=0;
	Vector3 oldAxis(0.0f);

	for(uint8_t eA=0;eA<numSelEdgesA;eA++) {
		const MeshEdge &edgeA = meshA->edges[selEdgesA[eA]];
		Vector3 dirA = normalize(vertsBA[edgeA.vertIndex[1]] - vertsBA[edgeA.vertIndex[0]]);

		for(uint8_t eB=0;eB<numSelEdgesB;eB++) {
			const MeshEdge &edgeB = meshB->edges[selEdgesB[eB]];
			Vector3 dirB = normalize(meshB->verts[edgeB.vertIndex[1]] - meshB->verts[edgeB.vertIndex[0]]);

			// エッジが平行であれば判定しない
			if(isSameDirection(dirA,dirB)) continue;

			Vector3 axis = normalize(cross(dirA,dirB));
			
			if(isSameDirection(oldAxis,axis)) continue;

			// 分離軸の方向をチェック
			if(dot(meshB->facets[minFacetB].normal,axis) < 0.0f)
				axis = -axis;

			// ConvexBを分離軸に投影して範囲を取得
			float BMin=FLT_MAX,BMax=-FLT_MAX;
			getProjAxis(meshB->verts,selVertsB,numSelVertsB,axis,BMin,BMax);

			// ConvexAを分離軸に投影して範囲を取得
			float AMin=FLT_MAX,AMax=-FLT_MAX;
			getProjAxis(vertsBA,selVertsA,numSelVertsA,axis,AMin,AMax);

			// 判定

			// ■ 非接触

			// A:          +----+
			// B:+----+
			if(BMax <= AMin) {
				return AMin-BMax;
			}


			// ■ 内包

			//A:      +--+
			//B:   +------+
			if(BMin < AMin && AMax < BMax) {
				float d = (AMin-BMax) * depthRatio;
				if(distMin < d) {
					distMin = d;
					oldAxis = axis;
					SATtype = CROSS_AXIS;
					edgeIdA = edgeA.dirGroup;
					edgeIdB = edgeB.dirGroup;
				}
			}

			//A:   +------+
			//B:    +--+
			else if(AMin < BMin && BMax < AMax) {
				float d = (AMin-BMax) * depthRatio;
				if(distMin < d) {
					distMin = d;
					oldAxis = axis;
					SATtype = CROSS_AXIS;
					edgeIdA = edgeA.dirGroup;
					edgeIdB = edgeB.dirGroup;
				}
			}

			// ■ 接触
			
			// A:   +----+
			// B:+----+
			else if(BMin < AMin && BMax < AMax) {
				float d = (AMin-BMax) * depthRatio;
				if(distMin < d) {
					distMin = d;
					oldAxis = axis;
					SATtype = CROSS_AXIS;
					edgeIdA = edgeA.dirGroup;
					edgeIdB = edgeB.dirGroup;
				}
			}

			// A:   +----+
			// B:      +----+
			else if(AMin < BMin && AMax < BMax) {
				float d = (BMin-AMax) * depthRatio;
				if(distMin < d) {
					distMin = d;
					oldAxis = -axis;
					SATtype = CROSS_AXIS;
					edgeIdA = edgeA.dirGroup;
					edgeIdB = edgeB.dirGroup;
				}
			}
		}
	}

#ifdef ENABLE_PERF
	pc.countEnd();
#endif

	if(SATtype == CROSS_AXIS) {
		normal = transformB.getUpper3x3() * oldAxis;
	}
	else if(SATtype == A_AXIS){
		normal = transformA.getUpper3x3() * axisMin;
	}
	else {
		normal = transformB.getUpper3x3() * axisMin;
	}

	//-------------------------------------------
	// 最近接点の探索

#ifdef ENABLE_PERF
	pc.countBegin("CC:Pre Closest");
#endif

	// 分離軸方向に引き離す（最近接を判定するため、交差を回避させる）
	Vector3 sepAxisA = 1.1f * fabsf(distMin) * ( transpose(transformA.getUpper3x3()) * normal ); 
	Vector3 sepAxisB = transformBA.getUpper3x3() * sepAxisA;

	for(uint8_t v=0;v<meshA->numVerts;v++) {
		vertsBA[v] = vertsBA[v] + sepAxisB;
	}
	for(uint8_t v=0;v<meshB->numVerts;v++) {
		vertsAB[v] = vertsAB[v] - sepAxisA;
	}

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CC:Closest Edges");
#endif

	// エッジ間の最短距離と座標値を算出
	// ※ConvexA座標系
	
	float distance,distanceMin = FLT_MAX;
	Vector3 sA(0.0f),sB(0.0f);
	
	if(SATtype == CROSS_AXIS) {
		// エッジを集める
		gatherEdgesFromGroup(edgeIdA,meshA,selFacetsA,numSelFacetsA,selEdgesA,numSelEdgesA);
		gatherEdgesFromGroup(edgeIdB,meshB,selFacetsB,numSelFacetsB,selEdgesB,numSelEdgesB);

		closestTwoEdges(
				meshA->verts,meshA->edges,selEdgesA,numSelEdgesA,
				vertsAB,meshB->edges,selEdgesB,numSelEdgesB,
				sA,sB,distance);

		pointA = Point3(sA);
		pointB = transformBA * Point3(sB + sepAxisA);
	}

	// 各頂点から各面への最短距離と座標値を算出

	if(SATtype == A_AXIS || SATtype == B_AXIS) {
#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CC:Closest VertA x FacetB");
#endif
		uint8_t selMinFacetsA[NUMMESHFACETS] = {0};
		uint8_t numSelMinFacetsA = 0;
		uint8_t selMinVertsA[NUMMESHVERTICES] = {0};
		uint8_t numSelMinVertsA = 0;
		uint8_t selMinFacetsB[NUMMESHFACETS] = {0};
		uint8_t numSelMinFacetsB = 0;
		uint8_t selMinVertsB[NUMMESHVERTICES] = {0};
		uint8_t numSelMinVertsB = 0;

		// 面と頂点を集める
		if(SATtype == A_AXIS) {
			const MeshFacet &facetA = meshA->facets[minFacetA];
			gatherFacetsFromFacetGroup(facetA.dirGroup,meshA,selFacetsA,numSelFacetsA,selMinFacetsA,numSelMinFacetsA);
			Plane planeA(facetA.normal,meshA->verts[facetA.vertIndices[0]]);
			uint8_t minVertB = getClosestPointProjPlane(vertsAB,selVertsB,numSelVertsB,planeA);
			gatherFacetsFromVertId(minVertB,meshB,selFacetsB,numSelFacetsB,selMinFacetsB,numSelMinFacetsB);
		}
		else {
			const MeshFacet &facetB = meshB->facets[minFacetB];
			gatherFacetsFromFacetGroup(facetB.dirGroup,meshB,selFacetsB,numSelFacetsB,selMinFacetsB,numSelMinFacetsB);
			Plane planeB(facetB.normal,meshB->verts[facetB.vertIndices[0]]);
			uint8_t minVertA = getClosestPointProjPlane(vertsBA,selVertsA,numSelVertsA,planeB);
			gatherFacetsFromVertId(minVertA,meshA,selFacetsA,numSelFacetsA,selMinFacetsA,numSelMinFacetsA);
		}

		gatherVerts(meshA,selMinFacetsA,numSelMinFacetsA,selMinVertsA,numSelMinVertsA);
		gatherVerts(meshB,selMinFacetsB,numSelMinFacetsB,selMinVertsB,numSelMinVertsB);

		// ConvexAの頂点 -> ConvexBの面
		// ※ConvexB座標系
		closestVertAndFacet(
				sepAxisB,
				vertsBA,selMinVertsA,numSelMinVertsA,
				meshB->verts,meshB->facets,selMinFacetsB,numSelMinFacetsB,
				sA,sB,distance);

		if(distance < distanceMin) {
			pointA = transformAB * Point3(sA - sepAxisB);
			pointB = Point3(sB);
			distanceMin = distance;
		}

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CC:Closest VertB x FacetA");
#endif

		// ConvexBの頂点 -> ConvexAの面
		// ※ConvexA座標系
		closestVertAndFacet(
				-sepAxisA,
				vertsAB,selMinVertsB,numSelMinVertsB,
				meshA->verts,meshA->facets,selMinFacetsA,numSelMinFacetsA,
				sB,sA,distance);

		if(distance < distanceMin) {
			pointA = Point3(sA);
			pointB = transformBA * Point3(sB + sepAxisA);
			distanceMin = distance;
		}

		// 2点を結ぶベクトルが分離軸と一致するかチェック
		Vector3 wA(transformA * pointA);
		Vector3 wB(transformB * pointB);
		if(dot(wB-wA,normal) < 0.99f) {
			gatherEdges(meshA,selFacetsA,numSelFacetsA,selEdgesA,numSelEdgesA);
			gatherEdges(meshB,selFacetsB,numSelFacetsB,selEdgesB,numSelEdgesB);

			closestTwoEdges(
					meshA->verts,meshA->edges,selEdgesA,numSelEdgesA,
					vertsAB,meshB->edges,selEdgesB,numSelEdgesB,
					sA,sB,distance);

			if(distance < distanceMin) {
				pointA = transformAB * Point3(sA - sepAxisB);
				pointB = Point3(sB);
			}
		}
	}

#ifdef ENABLE_PERF
	pc.countEnd();
#endif
	
	return distMin;
}

#endif

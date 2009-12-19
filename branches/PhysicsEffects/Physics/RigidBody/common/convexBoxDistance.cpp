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

#include "Box.h"
#include "TriMesh.h"
#include "intersectFunction.h"
#include "sat_mesh_utils.h"

#if 1   // USE GJK for Convex-Box collision

#include "PfxGJKSolver.h"
#include "PfxGJKSupportFunc.h"

float closestConvexBox( Vector3& normal, Point3 &pointA, Point3 &pointB,
						const ConvexMesh *meshA, const Transform3 & transformA,
						Box boxB, const Transform3 & transformB,
						float distanceThreshold )
{
	(void) distanceThreshold;

	PfxGJKSolver gjk((void*)meshA,(void*)&boxB,getSupportVertexConvex,getSupportVertexBox);

	return gjk.collide(normal,pointA,pointB,transformA,transformB,FLT_MAX);
}

#else   // USE SAT for Convex-Box collision

//#define ENABLE_PERF

// ボックスのエッジ定義 : boxEdge[axis][sign]
const MeshEdge boxEdge[3][2][4] = {
	{ // X
		{ // +
			{{6,7},0,0},
			{{7,3},0,0},
			{{3,2},0,0},
			{{2,6},0,0},
		},
		{ // -
			{{5,4},0,0},
			{{4,0},0,0},
			{{0,1},0,0},
			{{1,5},0,0},
		},
	},
	{ // Y
		{ // +
			{{4,7},0,0},
			{{7,6},0,0},
			{{6,5},0,0},
			{{5,4},0,0},
		},
		{ // -
			{{0,3},0,0},
			{{3,2},0,0},
			{{2,1},0,0},
			{{1,0},0,0},
		},
	},
	{ // Z
		{ // +
			{{5,6},0,0},
			{{6,2},0,0},
			{{2,1},0,0},
			{{1,5},0,0},
		},
		{ // -
			{{4,7},0,0},
			{{7,3},0,0},
			{{3,0},0,0},
			{{0,4},0,0},
		},
	},
};

static inline
void distancePointAndAABB(const Vector3 &p,const Vector3 &aabb,Vector3 &s)
{
	s = p;
	s = maxPerElem(s,-aabb);
	s = minPerElem(s,aabb);
}

static
bool closestVertAndFacet(
	const Vector3 &sepAxis,													// 分離軸
	const Vector3 *vertsA,uint8_t numVertsA,								// Aの頂点情報
	const Vector3 *vertsB,const MeshFacet *facetsB,const uint8_t *selFacetsB,uint8_t numSelFacetsB,	// Bの面情報
	Vector3 &sA,Vector3 &sB,// 出力：最近接点
	float &distanceMin		// 出力：最近接距離
	)
{
	bool check = false;
	float depthMin = FLT_MAX;
	Vector3 minA(0.0f),minB(0.0f);

	for(uint8_t v=0;v<numVertsA;v++) {
		Vector3 vert = vertsA[v];
		
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
	const Vector3 *vertsA,const MeshEdge *edgesA,const uint8_t *selEdges,uint8_t numSelEdges,	// Aのエッジ
	const Vector3 *vertsB,const MeshEdge *edgesB,uint8_t numEdgesB,	// Bのエッジ
	Vector3 &sA,Vector3 &sB,	// 出力：最近接点
	float &distanceMin			// 出力：最近接距離
	)
{
	bool check = false;
	float depthMin = FLT_MAX;
	Vector3 minA(0.0f),minB(0.0f);

	for(uint8_t eA=0;eA<numSelEdges;eA++) {
		const MeshEdge &edgeA = edgesA[selEdges[eA]];
				
		for(uint8_t eB=0;eB<numEdgesB;eB++) {
			const MeshEdge &edgeB = edgesB[eB];
			
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

float closestConvexBox( Vector3& normal, Point3 &pointA, Point3 &pointB,
						const TriMesh *meshA, const Transform3 & transformA,
						Box boxB, const Transform3 & transformB,
						float distanceThreshold )
{
	enum {A_AXIS, B_AXIS, CROSS_AXIS} SATtype;
	uint8_t minFacetA=0;

	int minFacetB = 0;
	int minSignB = 0;

#ifdef ENABLE_PERF
	PerfCounter pc;
#endif

	Transform3 transformAB, transformBA;
	Matrix3 matrixAB, matrixBA;
	Vector3 offsetAB, offsetBA;

	// Bローカル→Aローカルへの変換
	transformAB = orthoInverse(transformA) * transformB;

	// Aローカル→Bローカルへの変換
	transformBA = orthoInverse(transformAB);

	matrixAB = transformAB.getUpper3x3();
	offsetAB = transformAB.getTranslation();
	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	Vector3 boxPnt[8] = {
		Vector3(-boxB.half[0],-boxB.half[1],-boxB.half[2]),
		Vector3(-boxB.half[0],-boxB.half[1], boxB.half[2]),
		Vector3( boxB.half[0],-boxB.half[1], boxB.half[2]),
		Vector3( boxB.half[0],-boxB.half[1],-boxB.half[2]),
		Vector3(-boxB.half[0], boxB.half[1],-boxB.half[2]),
		Vector3(-boxB.half[0], boxB.half[1], boxB.half[2]),
		Vector3( boxB.half[0], boxB.half[1], boxB.half[2]),
		Vector3( boxB.half[0], boxB.half[1],-boxB.half[2]),
	};

	//-------------------------------------------
	// 判定する面を絞り込む

	uint8_t selFacets[NUMMESHFACETS] = {0};
	uint8_t selVerts[NUMMESHVERTICES] = {0};
	uint8_t selEdges[NUMMESHEDGES] = {0};
	uint8_t numSelFacets = 0;
	uint8_t numSelVerts = 0;
	uint8_t numSelEdges = 0;

	Vector3 vertsBA[NUMMESHVERTICES]; // BローカルのA座標
	Vector3 vertsAB[8]; // AローカルのB座標

	// 座標をそれぞれのローカル系に変換する
	for(uint8_t v=0;v<meshA->numVerts;v++) {
		vertsBA[v] = offsetBA + matrixBA * meshA->verts[v];
	}
	
	for(uint8_t v=0;v<8;v++) {
		vertsAB[v] = offsetAB + matrixAB * boxPnt[v];
	}

#ifdef ENABLE_PERF
	pc.countBegin("CB:Cull Faces");
#endif

	// ※boxB座標系
	gatherFacets(meshA,(float*)&boxB.half,offsetBA,matrixBA,selFacets,numSelFacets,selVerts,numSelVerts);
	
	if(numSelFacets == 0) {
		return distanceThreshold;
	}
	
	//-------------------------------------------
	// 分離軸判定(SAT)

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CB:SAT facet normal");
#endif

	// 最も浅い貫通深度とそのときの分離軸
	float distMinA = -FLT_MAX;
	float distMinB = -FLT_MAX;
	float distMin = -FLT_MAX;
	Vector3 axisMinA(0.0f);
	Vector3 axisMinB(0.0f);
	Vector3 axisMin(0.0f);

	// Convexの面 -> Box (Aローカル)
	// ※Box座標系
	for(uint8_t f=0;f<numSelFacets;f++) {
		const MeshFacet &facet = meshA->facets[selFacets[f]];

		// 分離軸
		const Vector3 axis = matrixBA * facet.normal;
		
		// 分離平面
		Plane planeA(axis,vertsBA[facet.vertIndices[0]]);
		
		// Boxを分離軸に投影して範囲を取得
		float r = dot(boxB.half,absPerElem(axis));
		float boxOffset = planeA.onPlane(Vector3(0.0f));
		float boxMin = boxOffset - r;
		
		// 判定
		if(boxMin > 0.0f) {
			return boxMin;
		}

		if(distMinA < boxMin) {
			distMinA = boxMin;
			axisMinA = -axis;
			minFacetA = selFacets[f];
		}
	}

	// Boxの面 -> Convex (Bローカル)
	// ※Box座標系
	for(uint8_t f=0;f<3;f++) {
		// 分離軸
		Vector3 axis(0.0f);
		axis[f] = 1.0f;

		// 分離平面
		Vector3 posB(0.0f);
		posB[f] = boxB.half[f];
		Plane planeB(axis,posB);

		float convexMin=FLT_MAX,convexMax=-FLT_MAX;
		float boxlen = 2.0f * boxB.half[f];
	
		// Convexを分離軸に投影して範囲を取得
		getProjPlane(vertsBA,selVerts,numSelVerts,planeB,convexMin,convexMax);

		// 判定
		if(convexMin > 0.0f) return convexMin;
		if(convexMax < -boxlen) return -boxlen-convexMax;

		if(distMinB < convexMin) {
			distMinB = convexMin;
			axisMinB = axis;
			minFacetB = f;
			minSignB = 0;
		}

		if(distMinB < (-boxlen-convexMax)) {
			distMinB = (-boxlen-convexMax);
			axisMinB = -axis;
			minFacetB = f;
			minSignB = 1;
		}
	}

	distMin = distMinA<distMinB?distMinB:distMinA;
	axisMin = distMinA<distMinB?axisMinB:axisMinA;
	SATtype = distMinA<distMinB?B_AXIS:A_AXIS;

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CB:SAT Edge x Edge");
#endif

	///////////////////////////////////////////////////////////////////////////////
	// エッジ Convexの全エッジ×Boxの全エッジ(x3)
	// ※Box座標系

	// エッジを集める
	gatherEdges2(meshA,selFacets,numSelFacets,selEdges,numSelEdges);

	// 同じ貫通深度であれば面判定のほうを優先させる
	const float depthRatio = 1.01f;

	uint8_t edgeIdA=0;
	Vector3 oldAxis(0.0f);

	for(uint8_t e=0;e<numSelEdges;e++) {
		const MeshEdge &edge = meshA->edges[selEdges[e]];
		Vector3 dir = normalize(vertsBA[edge.vertIndex[1]] - vertsBA[edge.vertIndex[0]]);

		for(int i=0;i<3;i++) {
			Vector3 edgeB(0.0f);
			edgeB[i] = 1.0f;
			
			// エッジが平行であれば判定しない
			if(isSameDirection(dir,edgeB)) continue;

			Vector3 axis = normalize(cross(dir,edgeB));

			if(isSameDirection(oldAxis,axis)) continue;

			// 分離軸の方向をチェック
			Vector3 boxNormal(0.0f);
			boxNormal[minFacetB] = minSignB==0?1.0:-1.0f;
			if(dot(boxNormal,axis) < 0.0f)
				axis = -axis;

			// Convexを分離軸に投影して範囲を取得
			float AMin=FLT_MAX,AMax=-FLT_MAX;
			getProjAxis(vertsBA,selVerts,numSelVerts,axis,AMin,AMax);

			// Boxを分離軸に投影して範囲を取得
			float r = dot(boxB.half,absPerElem(axis));
			float BMin = -r;
			float BMax = r;

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
					edgeIdA = edge.dirGroup;
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
					edgeIdA = edge.dirGroup;
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
					edgeIdA = edge.dirGroup;
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
					edgeIdA = edge.dirGroup;
				}
			}
		}
	}

	if(SATtype == CROSS_AXIS) {
		normal = transformB.getUpper3x3() * oldAxis;
	}
	else {
		normal = transformB.getUpper3x3() * axisMin;
	}

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CB:Find Contact V-F");
#endif

	//-------------------------------------------
	// 最近接点の探索

	// 分離軸方向に引き離す（最近接を判定するため、交差を回避させる）
	Vector3 sepAxisA = 1.1f * fabsf(distMin) * ( transpose(transformA.getUpper3x3()) * normal ); 
	Vector3 sepAxisB = transformBA.getUpper3x3() * sepAxisA;

	for(uint8_t v=0;v<meshA->numVerts;v++) {
		vertsBA[v] = vertsBA[v] + sepAxisB;
	}
	
	for(uint8_t v=0;v<8;v++) {
		vertsAB[v] = vertsAB[v] - sepAxisA;
	}

	// エッジ間の最短距離と座標値を算出
	// ※Convex座標系

	float distance,distanceMin = FLT_MAX;
	Vector3 sA(0.0f),sB(0.0f);
	
	if(SATtype == CROSS_AXIS) {
		// エッジを集める
		gatherEdgesFromGroup(edgeIdA,meshA,selFacets,numSelFacets,selEdges,numSelEdges);

		closestTwoEdges(
				meshA->verts,meshA->edges,selEdges,numSelEdges,
				vertsAB,boxEdge[minFacetB][minSignB],4,
				sA,sB,distance);

		pointA = Point3(sA);
		pointB = transformBA * Point3(sB + sepAxisA);
	}
	
	// Convexの頂点 -> Boxの面
	// ※Box座標系

	if(SATtype == A_AXIS || SATtype == B_AXIS) {
		uint8_t selMinFacetsA[NUMMESHFACETS] = {0};
		uint8_t numSelMinFacetsA = 0;
		uint8_t selMinVertsA[NUMMESHVERTICES] = {0};
		uint8_t numSelMinVertsA = 0;

		// 面と頂点を集める
		if(SATtype == A_AXIS) {
			const MeshFacet &facetA = meshA->facets[minFacetA];
			gatherFacetsFromFacetGroup(facetA.dirGroup,meshA,selFacets,numSelFacets,selMinFacetsA,numSelMinFacetsA);
		}
		else {
			Vector3 axis(0.0f);
			axis[minFacetB] = minSignB==0?1.0f:-1.0f;

			Vector3 posB(0.0f);
			posB[minFacetB] = axis[minFacetB] * boxB.half[minFacetB];

			Plane planeB(axis,posB);
			uint8_t minVertA = getClosestPointProjPlane(vertsBA,selVerts,numSelVerts,planeB);
			gatherFacetsFromVertId(minVertA,meshA,selFacets,numSelFacets,selMinFacetsA,numSelMinFacetsA);
		}

		gatherVerts(meshA,selMinFacetsA,numSelMinFacetsA,selMinVertsA,numSelMinVertsA);

		for(uint8_t v=0;v<numSelMinVertsA;v++) {
			Vector3 p = vertsBA[selMinVertsA[v]];
			Vector3 s;
			distancePointAndAABB(p,boxB.half,s);
			distance = lengthSqr(s-p);
			if(distance < distanceMin) {
				sA = p;
				sB = s;
				distanceMin = distance;
				pointA = transformAB * Point3(sA - sepAxisB);
				pointB = Point3(sB);
			}
		}

		// Boxの頂点 -> Convexの面
		// ※Convex座標系

		closestVertAndFacet(
				-sepAxisA,
				vertsAB,8,
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
			gatherEdges(meshA,selFacets,numSelFacets,selEdges,numSelEdges);

			closestTwoEdges(
					meshA->verts,meshA->edges,selEdges,numSelEdges,
					vertsAB,boxEdge[minFacetB][minSignB],4,
					sA,sB,distance);

			if(distance < distanceMin) {
				pointA = Point3(sA);
				pointB = transformBA * Point3(sB + sepAxisA);
			}
		}
	}

#ifdef ENABLE_PERF
	pc.countEnd();
#endif

	return distMin;
}

#endif

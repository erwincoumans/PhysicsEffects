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
#include "TriMesh.h"
#include "intersectFunction.h"
#include "PfxContactCache.h"
#include "sat_mesh_utils.h"
#include "SubData.h"

//#define ENABLE_PERF

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
	else if(BMin < AMin && AMin < BMax) {// Todo:SIMD
		float d = AMin-BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	}

	// A:   +----+
	// B:      +----+
	else if(AMin < BMin && BMin < AMax) {// Todo:SIMD
		float d = BMin-AMax;
		if(distMin < d) {
			distMin = d;
			axisMin = -axis;
		}
	}

	return false;
}

bool triangleBoxContact(Vector3 &axis,Vector3 &pointsOnTriangle,Vector3 &pointsOnBox,
							const Vector3 &normal,const Vector3 &p0,const Vector3 &p1,const Vector3 &p2,const float thickness,uint32_t edgeChk,
							const Vector3 &boxHalf)
{
	// 最も浅い貫通深度とそのときの分離軸
	float distMin = -FLT_MAX;
	Vector3 axisMin(0.0f);

	//-------------------------------------------
	// １．分離軸判定
	{
		Vector3 facetPnts[6] = {
			p0,p1,p2,p0-thickness*normal,p1-thickness*normal,p2-thickness*normal
		};

		// Trianglesの面 -> Box
		{
			// 分離軸
			const Vector3 sepAxis = normal;

			// 分離平面
			Plane planeA(sepAxis,p0);
			
			// Boxを分離軸に投影して範囲を取得
			float r = dot(boxHalf,absPerElem(sepAxis));
			float boxOffset = planeA.onPlane(Vector3(0.0f));
			float boxMax = boxOffset + r;
			float boxMin = boxOffset - r;
			
			// 判定
			if(boxMin > 0.0f || boxMax < -thickness) {
				return false;
			}

			if(distMin < boxMin) {
				distMin = boxMin;
				axisMin = -sepAxis;
			}
		}

		// Box -> Triangles
		for(uint8_t bf=0;bf<3;bf++) {
			// 分離軸
			Vector3 sepAxis(0.0f);
			sepAxis[bf] = 1.0f;

			// 分離軸の方向をチェック
			if(dot(normal,sepAxis) > 0.0f)
				sepAxis = -sepAxis;

			// Trianglesを分離軸に投影して範囲を取得
			float triMin,triMax;
			getProjAxisPnts6(facetPnts,sepAxis,triMin,triMax);

			// Boxを分離軸に投影して範囲を取得
			float boxMin = -boxHalf[bf];
			float boxMax =  boxHalf[bf];
			
			if(checkSAT(sepAxis,triMin,triMax,boxMin,boxMax,distMin,axisMin)) {
				return false;
			}
		}

		// エッジ Triangles面のエッジ(x3)×Boxのエッジ(x3)
		for(uint8_t e=0;e<3;e++) {
			Vector3 dir = normalize(facetPnts[(e+1)%3] - facetPnts[e]);

			for(int i=0;i<3;i++) {
				Vector3 boxEdge(0.0f);
				boxEdge[i] = 1.0f;
				
				// エッジが平行であれば判定しない
				if(isSameDirection(dir,boxEdge)) continue;

				Vector3 sepAxis = normalize(cross(dir,boxEdge));

				// 分離軸の方向をチェック
				if(dot(normal,sepAxis) > 0.0f)
					sepAxis = -sepAxis;

				// Trianglesを分離軸に投影して範囲を取得
				float triMin,triMax;
				getProjAxisPnts6(facetPnts,sepAxis,triMin,triMax);

				// Boxを分離軸に投影して範囲を取得
				float r = dot(boxHalf,absPerElem(sepAxis));
				float boxMin = -r;
				float boxMax = r;

				if(checkSAT(sepAxis,triMin,triMax,boxMin,boxMax,distMin,axisMin)) {
					return false;
				}
			}
		}

		// 面に厚みがある場合の補助的な判定（面法線とBoxのエッジ(x3)）
		// 交差するかしないかだけを判定
		if(thickness > 0.0f) {
			for(int i=0;i<3;i++) {
				Vector3 boxEdge(0.0f);
				boxEdge[i] = 1.0f;
				
				// エッジが平行であれば判定しない
				if(isSameDirection(normal,boxEdge)) continue;

				Vector3 sepAxis = normalize(cross(normal,boxEdge));

				// Trianglesを分離軸に投影して範囲を取得
				float triMin,triMax;
				getProjAxisPnts6(facetPnts,sepAxis,triMin,triMax);

				// Boxを分離軸に投影して範囲を取得
				float r = dot(boxHalf,absPerElem(sepAxis));
				float boxMin = -r;
				float boxMax = r;

				if(boxMax <= triMin || triMax <= boxMin) {
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
		
		const Vector3 boxPnts[8] = {
			Vector3(-boxHalf[0],-boxHalf[1],-boxHalf[2]),
			Vector3(-boxHalf[0],-boxHalf[1], boxHalf[2]),
			Vector3( boxHalf[0],-boxHalf[1], boxHalf[2]),
			Vector3( boxHalf[0],-boxHalf[1],-boxHalf[2]),
			Vector3(-boxHalf[0], boxHalf[1],-boxHalf[2]),
			Vector3(-boxHalf[0], boxHalf[1], boxHalf[2]),
			Vector3( boxHalf[0], boxHalf[1], boxHalf[2]),
			Vector3( boxHalf[0], boxHalf[1],-boxHalf[2]),
		};

		//--------------------------------------------------------------------
		// 衝突点の探索

		float depthSqrMin = FLT_MAX;
		Vector3 sA,sB;
		
		// エッジ間の最短距離と座標値を算出
		{
			int boxIds[] = {
				0,1,
				1,2,
				2,3,
				3,0,
				4,5,
				5,6,
				6,7,
				7,4,
				0,4,
				3,7,
				2,6,
				1,5,
			};

			for(int i=0;i<3;i++) {
				for(int j=0;j<12;j++) {
					distanceTwoLines(facetPnts[i],facetPnts[(i+1)%3],boxPnts[boxIds[j*2]],boxPnts[boxIds[j*2+1]],sA,sB);
					float distSqr = lengthSqr(sA-sB);
					if(distSqr < depthSqrMin) {
						depthSqrMin = distSqr;
						pointsOnTriangle = sA;
						pointsOnBox = sB;
					}
				}
			}
		}
		
		// Triangleの頂点 -> Boxの面
		{
			closestPointAndAABB(facetPnts[0],boxHalf,sB);
			float distSqr = lengthSqr(sB-facetPnts[0]);
			if(distSqr < depthSqrMin) {
				depthSqrMin = distSqr;
				pointsOnTriangle = facetPnts[0];
				pointsOnBox = sB;
			}
			closestPointAndAABB(facetPnts[1],boxHalf,sB);
			distSqr = lengthSqr(sB-facetPnts[1]);
			if(distSqr < depthSqrMin) {
				depthSqrMin = distSqr;
				pointsOnTriangle = facetPnts[1];
				pointsOnBox = sB;
			}
			closestPointAndAABB(facetPnts[2],boxHalf,sB);
			distSqr = lengthSqr(sB-facetPnts[2]);
			if(distSqr < depthSqrMin) {
				depthSqrMin = distSqr;
				pointsOnTriangle = facetPnts[2];
				pointsOnBox = sB;
			}
		}
		
		// Boxの頂点 -> Trianglesの面
		for(int i=0;i<8;i++) {
			distancePointAndTriangle(facetPnts[0],facetPnts[1],facetPnts[2],boxPnts[i],sA);
			float distSqr = lengthSqr(sA-boxPnts[i]);
			if(distSqr < depthSqrMin) {
				depthSqrMin = distSqr;
				pointsOnTriangle = sA;
				pointsOnBox = boxPnts[i];
			}
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

// 分離軸が見つかった場合はすぐに抜けるため最短距離が返されるわけではないことに注意
int trianglesBoxContacts( Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, float *distance,
								   const TriMesh *meshA, const Transform3 & transformA,
								   Box boxB, const Transform3 & transformB,float distanceThreshold)
{
	(void) distanceThreshold;

	Transform3 transformAB, transformBA;
	Matrix3 matrixBA;
	Vector3 offsetBA;

	// Bローカル→Aローカルへの変換
	transformAB = orthoInverse(transformA) * transformB;

	// Aローカル→Bローカルへの変換
	transformBA = orthoInverse(transformAB);

	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	//-------------------------------------------
	// 判定する面を絞り込む

	uint8_t selFacets[NUMMESHFACETS] = {0};
	uint8_t numSelFacets = 0;

	// ※boxB座標系
	Vector3 aabbB = boxB.half;
	gatherFacets(meshA,(float*)&aabbB,offsetBA,matrixBA,selFacets,numSelFacets);

	if(numSelFacets == 0) {
		return 0;
	}
	
	//-------------------------------------------
	// 分離軸判定(SAT)
	// ※Box座標系 (Bローカル)で判定
	
	PfxContactCache contacts;
	
	for(uint32_t f = 0; f < numSelFacets; f++) {
		const MeshFacet &facet = meshA->facets[selFacets[f]];
		
		Vector3 facetNormal = matrixBA * read_Vector3(facet.normal);

		Vector3 facetPntsA[3] = {
			offsetBA + matrixBA * meshA->verts[facet.vertIndices[0]],
			offsetBA + matrixBA * meshA->verts[facet.vertIndices[1]],
			offsetBA + matrixBA * meshA->verts[facet.vertIndices[2]],
		};
		
		Vector3 axis,pointsOnTriangle,pointsOnBox;

		uint32_t edgeChk = 
			((meshA->edges[facet.edgeIndices[0]].angle==EDGE_CONVEX)?0x00:0x01) |
			((meshA->edges[facet.edgeIndices[1]].angle==EDGE_CONVEX)?0x00:0x02) |
			((meshA->edges[facet.edgeIndices[2]].angle==EDGE_CONVEX)?0x00:0x04);

		if(triangleBoxContact(axis,pointsOnTriangle,pointsOnBox,
							facetNormal,facetPntsA[0],facetPntsA[1],facetPntsA[2],facet.thickness,edgeChk,
							boxB.half)) {
			// 衝突点を追加する
			contacts.add(
				-length(pointsOnBox - pointsOnTriangle),
				axis,
				pointsOnTriangle,
				pointsOnBox,
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

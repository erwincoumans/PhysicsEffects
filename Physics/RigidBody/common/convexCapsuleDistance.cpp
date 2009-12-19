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
#include "Capsule.h"
#include "TriMesh.h"
#include "intersectFunction.h"
#include "sat_mesh_utils.h"

#if 1   // USE GJK for Convex-Capsule collision

#include "PfxGJKSolver.h"
#include "PfxGJKSupportFunc.h"

float closestConvexCapsule(Vector3 &normal, Point3 &pointA, Point3 &pointB,
						const ConvexMesh *meshA, const Transform3 &transformA,
						Capsule capB, const Transform3 &transformB,
						float distanceThreshold)
{
	(void) distanceThreshold;
	
	PfxGJKSolver gjk((void*)meshA,(void*)&capB,getSupportVertexConvex,getSupportVertexCapsule);

	return gjk.collide(normal,pointA,pointB,transformA,transformB,FLT_MAX);
}

#else   // USE SAT for Convex-Capsule collision

//#define ENABLE_PERF

float closestConvexCapsule(Vector3 &normal, Point3 &pointA, Point3 &pointB,
						const TriMesh *meshA, const Transform3 &transformA,
						Capsule capB, const Transform3 &transformB,
						float distanceThreshold)
{
	uint8_t minFacet=0;

#ifdef ENABLE_PERF
	PerfCounter pc;
#endif

	Transform3 transformAB, transformBA;
	Matrix3 matrixAB, matrixBA;
	Vector3 offsetAB, offsetBA;

	// Bローカル→Aローカルへの変換.
	transformAB = orthoInverse(transformA) * transformB;

	// Aローカル→Bローカルへの変換.
	transformBA = orthoInverse(transformAB);

	matrixAB = transformAB.getUpper3x3();
	offsetAB = transformAB.getTranslation();
	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	// カプセルのAABBを作成.
	Vector3 vCapLocalAxis(1.0f, 0.0f, 0.0f);
	Box	boxB = capB.GetAABB(vCapLocalAxis);

	//-------------------------------------------
	// 判定する面を絞り込む.

	uint8_t selFacets[NUMMESHFACETS] = { 0 };
	uint8_t selVerts[NUMMESHVERTICES] = { 0 };
	uint8_t selEdges[NUMMESHEDGES] = { 0 };
	uint8_t numSelFacets = 0;
	uint8_t numSelVerts = 0;
	uint8_t numSelEdges = 0;

	Vector3 vertsBA[NUMMESHVERTICES]; // BローカルのA座標

	// 座標をそれぞれのローカル系に変換する
	for(uint8_t v=0;v<meshA->numVerts;v++) {
		vertsBA[v] = offsetBA + matrixBA * meshA->verts[v];
	}

#ifdef ENABLE_PERF
	pc.countBegin("CC:Cull Faces");
#endif

	// ※capsuleB座標系.
	gatherFacets(meshA,(float*)&boxB.half,offsetBA,matrixBA,selFacets,numSelFacets,selVerts,numSelVerts);

	if(numSelFacets == 0) {
		return distanceThreshold;
	}

	//-------------------------------------------
	// 分離軸判定(SAT)

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CC:SAT facet normal");
#endif

	// 最も浅い貫通深度とそのときの分離軸.
	float distMin = -FLT_MAX;
	Vector3 axisMin(0.0f);

	// Convexの面 -> Capsule (Aローカル)
	// ※Convex座標系.
	Vector3 vCap0 = offsetAB + matrixAB * Vector3(-capB.hLength, 0.0f, 0.0f);
	Vector3 vCap1 = offsetAB + matrixAB * Vector3( capB.hLength, 0.0f, 0.0f);
	for (uint8_t f = 0; f < numSelFacets; f++) {
		const MeshFacet &facet = meshA->facets[selFacets[f]];

		// 分離軸.
		const Vector3 &axis = facet.normal;

		// 分離平面
		Plane planeA(axis,meshA->verts[facet.vertIndices[0]]);

		// カプセルを分離軸に投影して範囲を取得.
		float capOffset0 = planeA.onPlane(vCap0);
		float capOffset1 = planeA.onPlane(vCap1);
		float capMin;
		if (capOffset0 < capOffset1) {
			capMin = capOffset0 - capB.radius;
		} else {
			capMin = capOffset1 - capB.radius;
		}

		// 判定
		if(capMin > 0.0f) {
			return capMin;
		}

		if(distMin < capMin) {
			distMin = capMin;
			axisMin = transformA.getUpper3x3() * (-axis);
			minFacet = selFacets[f];
		}
	}

	// Capsule -> Convexの面 (Bローカル)
	// ※Capsule座標系.

	for (uint8_t f = 0; f < 2; f++) {
		// Capsuleを分離軸に投影して範囲を取得.
		float BMin,BMax;
		Vector3 axis;
		switch(f) {
		case	0:	// カプセルの軸が分離軸.
			axis = vCapLocalAxis;
			BMax = capB.hLength + capB.radius;
			if(dot(matrixBA*meshA->facets[minFacet].normal,axis) > 0.0f)
				axis = -axis;
			break;
		default:	// Convexからカプセルの軸への垂線が分離軸.
			axis = normalize(Vector3(0.0f, offsetBA[1], offsetBA[2]));
			BMax = capB.radius;
			break;
		}
		BMin = -BMax;

		// Convexを分離軸に投影して範囲を取得.
		float AMin=FLT_MAX,AMax=-FLT_MAX;
		getProjAxis(vertsBA,selVerts,numSelVerts,axis,AMin,AMax);

		// 判定.

		// ■ 非接触.

		// A:          +----+
		// B:+----+
		if (BMax <= AMin) {
			return AMin - BMax;
		}


			// ■ 内包

			//A:      +--+
			//B:   +------+
			if(BMin < AMin && AMax < BMax) {
				float d = AMin-BMax;
				if(distMin < d) {
					distMin = d;
					axisMin = transformB.getUpper3x3() * axis;
				}
			}

			//A:   +------+
			//B:    +--+
			else if(AMin < BMin && BMax < AMax) {
				float d = AMin-BMax;
				if(distMin < d) {
					distMin = d;
					axisMin = transformB.getUpper3x3() * axis;
				}
			}

			// ■ 接触
			
			// A:   +----+
			// B:+----+
			else if(BMin < AMin && BMax < AMax) {
				float d = AMin-BMax;
				if(distMin < d) {
					distMin = d;
					axisMin = transformB.getUpper3x3() * axis;
				}
			}

			// A:   +----+
			// B:      +----+
			else if(AMin < BMin && AMax < BMax) {
				float d = BMin-AMax;
				if(distMin < d) {
					distMin = d;
					axisMin = transformB.getUpper3x3() * -axis;
				}
			}
	}

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CC:SAT Edge x Capsule");
#endif

	///////////////////////////////////////////////////////////////////////////////
	// エッジ Convexの全エッジ×Capsule
	// ※Convex座標系.

	// エッジを集める
	gatherEdges2(meshA,selFacets,numSelFacets,selEdges,numSelEdges);

	// 同じ貫通深度であれば面判定のほうを優先させる
	const float depthRatio = 1.01f;

	if(0.00001f < capB.hLength) {
		// 筒がある.
		Vector3 vCapEdge = matrixAB * vCapLocalAxis; // Aのローカル座標系へ変換.

		for (uint8_t e = 0; e < numSelEdges; e++) {
			const MeshEdge &edge = meshA->edges[selEdges[e]];

			Vector3 startPos = meshA->verts[edge.vertIndex[0]];
			Vector3 endPos = meshA->verts[edge.vertIndex[1]];
			Vector3 dir = normalize(endPos - startPos);

			// エッジが平行であれば判定しない
			if(isSameDirection(dir,vCapEdge)) continue;

			Vector3 axis = normalize(cross(dir, vCapEdge));

			// 分離軸の方向をチェック
			if(dot(meshA->facets[minFacet].normal,axis) > 0.0f)
				axis = -axis;

			// Convexを分離軸に投影して範囲を取得.
			float AMin=FLT_MAX,AMax=-FLT_MAX;
			getProjAxis(meshA->verts,selVerts,numSelVerts,axis,AMin,AMax);

			// カプセルを分離軸に投影して範囲を取得.
			float r = capB.radius;
			float capOffset = dot(offsetAB, axis);
			float BMin = capOffset - r;
			float BMax = capOffset + r;

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
					axisMin = transformA.getUpper3x3() * axis;
				}
			}

			//A:   +------+
			//B:    +--+
			else if(AMin < BMin && BMax < AMax) {
				float d = (AMin-BMax) * depthRatio;
				if(distMin < d) {
					distMin = d;
					axisMin = transformA.getUpper3x3() * axis;
				}
			}

			// ■ 接触
			
			// A:   +----+
			// B:+----+
			else if(BMin < AMin && BMax < AMax) {
				float d = (AMin-BMax) * depthRatio;
				if(distMin < d) {
					distMin = d;
					axisMin = transformA.getUpper3x3() * axis;
				}
			}

			// A:   +----+
			// B:      +----+
			else if(AMin < BMin && AMax < BMax) {
				float d = (BMin-AMax) * depthRatio;
				if(distMin < d) {
					distMin = d;
					axisMin = transformA.getUpper3x3() * -axis;
				}
			}
		}
	}

	normal = axisMin;

#ifdef ENABLE_PERF
	pc.countEnd();
	pc.countBegin("CC:Find Contact V-F");
#endif

	//----------------------------------------------------------------
	// 最近接点の検索

	// 分離軸方向に引き離す(最近接を判定するため、交差回避させる)
	Vector3 sepAxisA = 1.1f * fabsf(distMin) * (transpose(transformA.getUpper3x3()) * normal);
	Vector3 sepAxisB = transformBA.getUpper3x3() * sepAxisA;

	for (uint8_t v = 0; v < meshA->numVerts; v++) {
		vertsBA[v] = vertsBA[v] + sepAxisB;
	}

	float depthMin = FLT_MAX;
	Vector3 sA,sB;
	Point3 ssA(0.0f),ssB(0.0f);
	bool check = false;

	//--------------------------------------------------------------------
	// 衝突点の探索.

	//--------------------------------------------------------------------
	// Convexの頂点 -> Capsule
	// ※Capsule座標系.

	// capsuleの半径の2乗
	for (uint8_t v = 0; v < numSelVerts; v++) {
		// convexの頂点を引き離した後の情報
		float d;

		Vector3 p = vertsBA[selVerts[v]];
		float fp = fabsf(dot(vCapLocalAxis, p));
		float fpA = fabsf(dot(vCapLocalAxis, sepAxisB));

		if (fp - fpA < capB.hLength) {
			// 円筒との接触判定.
			float fLenSq = lengthSqr(Vector3(0.0f, p[1], p[2]));
			d = sqrtf(fLenSq) - capB.radius;
			if (d < depthMin) {
				Point3 cp(p);
				ssA = transformAB * (cp - sepAxisB);
				ssB = cp;
				depthMin = d;
				check = true;
			}
//		} else if (fp < capB.hLength + capB.radius) {
		} else if (fp - fpA < capB.hLength + capB.radius) {
			// 半球との接触判定.
			float fLenSq = lengthSqr(Vector3(fabsf(p[0]) - capB.hLength, p[1], p[2]));
			d = sqrtf(fLenSq) - capB.radius;
			if( d < depthMin) {
				Point3 cp(p);
				ssA = transformAB * (cp - sepAxisB);
				ssB = cp;
				depthMin = d;
				check = true;
			}
		}
	}

	if (check) {
		pointA = ssA;
		pointB = ssB;;
	}

	//-----------------------------------------------------------
	// Capsule -> Convexの面.
	// ※Convex座標系.

	check = false;
	{
		// カプセルの始点/終点の座標
		Vector3 capPnt[2] = {
			Vector3(-capB.hLength, 0.0f, 0.0f),
			Vector3( capB.hLength, 0.0f, 0.0f),
		};

		for (uint8_t f = 0; f < numSelFacets; f++) {
			// Convexの面
			const MeshFacet &facet = meshA->facets[selFacets[f]];

			// 逆向きの面は排除.
			if (dot(offsetAB, facet.normal) < 0.0f) {
				continue;
			}

			// Convexの面
			Vector3 facetPnts[3] = {
				meshA->verts[facet.vertIndices[0]],
				meshA->verts[facet.vertIndices[1]],
				meshA->verts[facet.vertIndices[2]],
			};

			// Convex面の法線方向にcapsuleの半径分、移動させる
			Vector3 vn = offsetAB - facet.normal * capB.radius;
			// capsuleの始点位置の算出
			Vector3 p0 = vn + matrixAB * capPnt[0];
			// capsuleの終点位置の算出
			Vector3 p1 = vn + matrixAB * capPnt[1];
			Vector3 startPos;
			Vector3 dir = facet.normal;
			// 始点の位置を確定/引き離した後の位置の算出
			if (dot(p1 - p0, dir) < 0.0f) {
				startPos = p1 - sepAxisA;
			}
			else {
				startPos = p0 - sepAxisA;
			}

//			dir *= sum(absPerElem(((TriMesh *)meshA)->getAABB())) + capB.radius;

			// 最近接点の算出
			Vector3 closest;
			distancePointAndTriangle(facetPnts[0],facetPnts[1], facetPnts[2],startPos,closest);
			float d = lengthSqr(closest - startPos);
			d = sqrtf(d);
			if ( d < depthMin) {
				Point3 cp(startPos);
				ssA = cp;
				ssB = transformBA * (Point3(closest) + sepAxisA);
				depthMin = d;
				check = true;
			}

		}

		if(check) {
			pointA = ssA;
			pointB = ssB;
		}
	}


	//---------------------------------------------
	// Convexのエッジ -> Capsule
	// ※Capsule座標系.

	check = false;
	{
		gatherEdges(meshA,selFacets,numSelFacets,selEdges,numSelEdges);

		Vector3 capPnt[2] = {
			Vector3(-capB.hLength, 0.0f, 0.0f),
			Vector3( capB.hLength, 0.0f, 0.0f),
		};
		if (0.00001f < capB.hLength) {
			// 筒がある.
			for (uint8_t e = 0; e < numSelEdges; e++) {
				// Convexのエッジ
				const MeshEdge &edge = meshA->edges[selEdges[e]];

				// エッジの始点/終点の位置を引き伸ばした点を算出
				Vector3 edgeP = (offsetBA + matrixBA * meshA->verts[edge.vertIndex[0]]) + sepAxisB;
				Vector3 edgeQ = (offsetBA + matrixBA * meshA->verts[edge.vertIndex[1]]) + sepAxisB;
				Vector3 dir2 = edgeQ - edgeP;
				float dot2_2 = dot(dir2, dir2);

				if (dot2_2 < 0.00001f) {
					// 点とみなせる.
					continue;
				}

				if (1.0f - fabsf(dot(normalize(dir2), vCapLocalAxis)) < 0.00001f) {
					// 筒とエッジが平行.
					continue;
				}

				// 法線の算出
				Vector3 axis = normalize(cross(vCapLocalAxis, dir2));
				float fp = dot(axis, edgeP);
				float fLen = fabsf(fp);

	//			if (capB.radius < fLen) {
					// 筒(無限長)と交わらない.
	//				continue;
	//			}

				Vector3 capP = capPnt[0];
				Vector3 capQ = capPnt[1];
				Vector3 capR = axis * capB.radius;

				// capPQを筒の表面に移動.
				if (fLen < fabsf(dot(axis, edgeP + axis))) {
					capP += capR;
					capQ += capR;
				} else {
					capP -= capR;
					capQ -= capR;
				}

				// 線分と線分の最近接点S,Tを求める.(接しているかはみない)
				Vector3 dir1 = capQ - capP;
				Vector3 dir21 = capP - edgeP;
				float dot1_1 = dot(dir1, dir1);
				float dot1_2 = dot(dir1, dir2);
				float dn = dot1_1 * dot2_2 - dot1_2 * dot1_2;

				if (dn < 0.00001f) {
					continue;
				}

				float dot2_21 = dot(dir2, dir21);
				float dot1_21 = dot(dir1, dir21);
				float s = (dot1_2 * dot2_21 - dot1_21 * dot2_2) / dn;
				float t = dot1_2 * s + dot2_21;

				if (t < 0.0f || dot2_2 < t) {
					// エッジの外.
					continue;
				}

				t /= dot2_2;
				Vector3 edgeHit = edgeP + t * dir2;

				float d;
				if (s < 0.0f) {
					// capPの外.
					Vector3 v = edgeHit - capPnt[0];
					float lenSq = lengthSqr(v);
					d = sqrtf(lenSq) - capB.radius;
					if (d < depthMin) {
						Point3 cp = Point3(edgeHit);
						ssA = transformAB * (cp - sepAxisB);
						ssB = cp;
						depthMin = d;
						check = true;
					}
				} else if (1.0f < s) {
					// capQの外.
					Vector3 v = edgeHit - capPnt[1];
					float lenSq = lengthSqr(v);
					d = sqrtf(lenSq) - capB.radius;
					if (d < depthMin) {
						Point3 cp = Point3(edgeHit);
						ssA = transformAB * (cp - sepAxisB);
						ssB = cp;
						depthMin = d;
						check = true;
					}
				} else {
					d = fLen - capB.radius;
					if (d < depthMin) {
						Point3 cp = Point3(edgeHit);
						ssA = transformAB * (cp - sepAxisB);
						ssB = cp;
						depthMin = d; 
						check = true;
					}
				}
			}
		}

		if(check){
			pointA = ssA;
			pointB = ssB;
		}
	}

#ifdef ENABLE_PERF
	pc.countEnd();
#endif

	return distMin;
}

#endif

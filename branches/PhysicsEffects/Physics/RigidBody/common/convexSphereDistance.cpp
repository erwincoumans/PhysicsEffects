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

#include "Sphere.h"
#include "TriMesh.h"
#include "intersectFunction.h"
#include "sat_mesh_utils.h"

#if 1   // USE GJK for Convex-Sphere collision

#include "PfxGJKSolver.h"
#include "PfxGJKSupportFunc.h"

float closestConvexSphere( Vector3& normal,Point3 &pointA, Point3 &pointB,
						  const ConvexMesh *meshA, const Transform3 & transformA,
						  Sphere sphereB, const Transform3 & transformB,
						  float distanceThreshold)
{
	(void) distanceThreshold;

	PfxGJKSolver gjk((void*)meshA,(void*)&sphereB,getSupportVertexConvex,getSupportVertexSphere);

	return gjk.collide(normal,pointA,pointB,transformA,transformB,FLT_MAX);
}

#else   // USE SAT for Convex-Sphere collision

//#define ENABLE_PERF

float closestConvexSphere( Vector3& normal,Point3 &pointA, Point3 &pointB,
						  const ConvexMesh *meshA, const Transform3 & transformA,
						  Sphere sphereB, const Transform3 & transformB,
						  float distanceThreshold)
{
#ifdef ENABLE_PERF
	PerfCounter pc;
#endif

	Transform3 transformAB,transformBA;
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

	//------------------------------------------------
	// 判定面を絞り込む

	uint8_t selFacets[NUMMESHFACETS] = {0};
	uint8_t numSelFacets = 0;

	// ※sphereB座標系
	Vector3 rad(sphereB.radius);
	gatherFacets(meshA,(float*)&rad,offsetBA,matrixBA,selFacets,numSelFacets);

	if(numSelFacets == 0) {
		return distanceThreshold;
	}

	//-----------------------------------------------
	// 判定

	Vector3 axisMin(0.0f);
	float distMin = -FLT_MAX;

	// Convex面->sphereの判定
	// ※ConvexA座標系
	for(uint8_t f = 0; f < numSelFacets; f++ ) {
		const MeshFacet &facet = meshA->facets[selFacets[f]];

		Vector3 facetPnts[3] = {
			meshA->verts[facet.vertIndices[0]],	
			meshA->verts[facet.vertIndices[1]],	
			meshA->verts[facet.vertIndices[2]],	
		};
		
		Vector3 centerB = transformAB.getTranslation();
		Vector3 pntA;

		distancePointAndTriangle(facetPnts[0],facetPnts[1],facetPnts[2],centerB,pntA);
		
		// 球の中心から最近接点までのベクトル
		Vector3 distVec = pntA - centerB;
		float distVecSqrLen = lengthSqr(distVec);
		
		if(distVecSqrLen >= sphereB.radius * sphereB.radius) continue;

		// 球上の衝突点
		Vector3 sepAxis = distVecSqrLen < 0.00001f ? -facet.normal : distVec / sqrtf(distVecSqrLen);
		if(dot(sepAxis,facet.normal) > 0.0f) {
			sepAxis = -sepAxis;
		}

		Vector3 pntB = centerB + sphereB.radius * sepAxis;

		// 距離
		float d = dot((pntA-pntB),sepAxis);

		if(distMin < d) {
			distMin = d;
			axisMin = sepAxis;
			pointA = Point3(pntA);
			pointB = Point3(pntB);
		}
	}

	if(distMin == -FLT_MAX) return 0;

	normal = transformA.getUpper3x3()*axisMin;
	pointB = transformBA * pointB;

	return distMin;
}

#endif

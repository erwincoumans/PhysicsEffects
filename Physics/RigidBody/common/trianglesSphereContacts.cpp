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
#include "Physics/Base/SimdFunc.h"

#include "Sphere.h"
#include "TriMesh.h"
#include "intersectFunction.h"
#include "PfxContactCache.h"
#include "SubData.h"

bool triangleSphereContact(
	Vector3 &axis,Vector3 &pointsOnTriangle,Vector3 &pointsOnSphere,
	const Vector3 &normal,const Vector3 &p0,const Vector3 &p1,const Vector3 &p2,const float thickness,uint32_t edgeChk,
	float sphereRadius,const Vector3 &spherePos)
{
	Vector3 pntA;
	
	// 球と面の最近接点を計算
	distancePointAndTriangle(p0,p1,p2,spherePos,pntA);
	
	Vector3 distVec = pntA - spherePos;
	float l = length(distVec);

	// 分離軸
	Vector3 sepAxis = l < 0.00001f ? -normal : distVec / l;

	// 面の表裏判定
	bool backside = dot(sepAxis,normal) > 0.99f;

	if( (l>=sphereRadius && !backside) || (l>=PFX_MAX(thickness,sphereRadius) && backside)) return false;

	// 球上の衝突点
	axis = backside?-sepAxis:sepAxis;
	pointsOnSphere = spherePos + sphereRadius * axis;
	pointsOnTriangle = pntA;

	// 面上の最近接点が凸エッジ上でない場合は法線を変える
	if( ((edgeChk&0x01)&&pointOnLine(pointsOnTriangle,p0,p1)) ||
		((edgeChk&0x02)&&pointOnLine(pointsOnTriangle,p1,p2)) ||
		((edgeChk&0x04)&&pointOnLine(pointsOnTriangle,p2,p0)) ) {
		axis=-normal;
	}

	return true;
}

int trianglesSphereContacts( Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, float *distance,
								   const TriMesh *meshA, const Transform3 & transformA,
								   Sphere sphereB, const Transform3 & transformB,float distanceThreshold)
{
	(void) distanceThreshold;

	Transform3 transformAB;
	Transform3 transformBA;

	// Bローカル→Aローカルへの変換
	transformAB = orthoInverse(transformA) * transformB;

	// Aローカル→Bローカルへの変換
	transformBA = orthoInverse(transformAB);

	PfxContactCache contacts;

	//-----------------------------------------------
	// 判定

	// TriangleMeshの面->sphereの判定
	// ※TriangleMesh座標系
	{
		for(uint32_t f = 0; f < meshA->numFacets; f++ ) {
			const MeshFacet &facet = meshA->facets[f];

			Vector3 facetNormal = read_Vector3(facet.normal);

			Vector3 facetPnts[3] = {
				meshA->verts[facet.vertIndices[0]],	
				meshA->verts[facet.vertIndices[1]],	
				meshA->verts[facet.vertIndices[2]],	
			};
			
			Vector3 sepAxis,pntA,pntB;

			uint32_t edgeChk = 
				((meshA->edges[facet.edgeIndices[0]].angle==EDGE_CONVEX)?0x00:0x01) |
				((meshA->edges[facet.edgeIndices[1]].angle==EDGE_CONVEX)?0x00:0x02) |
				((meshA->edges[facet.edgeIndices[2]].angle==EDGE_CONVEX)?0x00:0x04);

			if(triangleSphereContact(sepAxis,pntA,pntB,
									facetNormal,facetPnts[0],facetPnts[1],facetPnts[2],facet.thickness,edgeChk,
									sphereB.radius,transformAB.getTranslation())) {
				contacts.add(
					-length(pntB - pntA),
					sepAxis,
					pntA,
					pntB,
					f);
			}
		}
	}

	// 面のローカル座標を算出
	for(uint32_t i=0;i<contacts.getNumContacts();i++) {
		normal[i] = transformA.getUpper3x3() * contacts.getNormal(i);
		pointA[i] = Point3(contacts.getPointA(i));
		pointB[i] = transformBA * Point3(contacts.getPointB(i));
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

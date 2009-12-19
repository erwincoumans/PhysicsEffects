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
#include "PfxContactCache.h"
#include "sat_mesh_utils.h"
#include "SubData.h"

#include "PfxGJKSolver.h"
#include "PfxGJKSupportFunc.h"

bool triangleConvexContact(Vector3 &axis,Vector3 &pointsOnTriangle,Vector3 &pointsOnConvex,
							const Vector3 &normal,const Vector3 &p0,const Vector3 &p1,const Vector3 &p2,const float thickness,uint32_t edgeChk,
							const ConvexMesh *meshB)
{
	const Vector3 facetPnts[6] = {
		p0,p1,p2,p0-thickness*normal,p1-thickness*normal,p2-thickness*normal
	};
	
	PfxGJKSolver gjk((void*)facetPnts,(void*)meshB,getSupportVertexTriangleWithThickness,getSupportVertexConvex);
	
	Point3 pA(0.0f),pB(0.0f);
	Vector3 nml(0.0f);
	
	float d = gjk.collide(nml,pA,pB,Transform3::identity(),Transform3::identity(),FLT_MAX);
	
	pointsOnTriangle = Vector3(pA);
	pointsOnConvex = Vector3(pB);
	axis = nml;
	
	// 面上の最近接点が凸エッジ上でない場合は法線を変える
	if( ((edgeChk&0x01)&&pointOnLine(pointsOnTriangle,p0,p1)) ||
		((edgeChk&0x02)&&pointOnLine(pointsOnTriangle,p1,p2)) ||
		((edgeChk&0x04)&&pointOnLine(pointsOnTriangle,p2,p0)) ) {
		axis=-normal;
	}

	return d < 0.0f;
}

int trianglesConvexContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, float *distance,
						const TriMesh *meshA, const Transform3 & transformA,
						const ConvexMesh *meshB, const Transform3 & transformB,float distanceThreshold )
{
	(void) distanceThreshold;

	Transform3 transformAB,transformBA;
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

	// ※Convex座標系
	Vector3 aabbB = read_Vector3(meshB->half);
	gatherFacets(meshA,(float*)&aabbB,offsetBA,matrixBA,selFacets,numSelFacets);

	if(numSelFacets == 0) {
		return 0;
	}
	
	//-------------------------------------------
	// 分離軸判定(SAT)
	// ※Convex座標系 (Bローカル)で判定
	
	PfxContactCache contacts;
	
	for(uint32_t f = 0; f < numSelFacets; f++) {
		const MeshFacet &facet = meshA->facets[selFacets[f]];
		
		Vector3 facetNormal = matrixBA * read_Vector3(facet.normal);

		Vector3 facetPntsA[3] = {
			offsetBA + matrixBA * meshA->verts[facet.vertIndices[0]],
			offsetBA + matrixBA * meshA->verts[facet.vertIndices[1]],
			offsetBA + matrixBA * meshA->verts[facet.vertIndices[2]],
		};
		
		Vector3 axis,pointsOnTriangle,pointsOnConvex;
		
		uint32_t edgeChk = 
			((meshA->edges[facet.edgeIndices[0]].angle==EDGE_CONVEX)?0x00:0x01) |
			((meshA->edges[facet.edgeIndices[1]].angle==EDGE_CONVEX)?0x00:0x02) |
			((meshA->edges[facet.edgeIndices[2]].angle==EDGE_CONVEX)?0x00:0x04);
		
		if(triangleConvexContact(axis,pointsOnTriangle,pointsOnConvex,
							facetNormal,facetPntsA[0],facetPntsA[1],facetPntsA[2],facet.thickness,edgeChk,
							meshB)) {
			// 衝突点を追加する
			contacts.add(
				-length(pointsOnConvex - pointsOnTriangle),
				axis,
				pointsOnTriangle,
				pointsOnConvex,
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

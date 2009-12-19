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

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/RigidBody/common/CollObject.h"
#include "Physics/RigidBody/common/intersectFunction.h"
#include "Physics/RigidBody/common/SubData.h"
#include "Physics/RayCast/common/Ray.h"
#include "Physics/RayCast/common/RayIntersectHeightField.h"



///////////////////////////////////////////////////////////////////////////////
// Ray Intesect Function Table

#define RAYINTERSECTFUNC(funcName) \
bool funcName(\
	const CollPrim &prim,\
	const Transform3 &transform,\
	Ray &ray,float &t );

typedef bool (*RayIntersect)(
	const CollPrim &prim,
	const Transform3 &transform,
	Ray &ray,float &t );

RAYINTERSECTFUNC(rayIntersectDummy)
RAYINTERSECTFUNC(rayIntersectSphere)
RAYINTERSECTFUNC(rayIntersectBox)
RAYINTERSECTFUNC(rayIntersectCapsule)
RAYINTERSECTFUNC(rayIntersectConvex)
RAYINTERSECTFUNC(rayIntersectLargeMesh)

RayIntersect funcTbl_rayIntersect[PRIM_COUNT] = {
	rayIntersectSphere,
	rayIntersectBox,
	rayIntersectCapsule,
	rayIntersectHeightField,
	rayIntersectConvex,
	rayIntersectDummy,
	rayIntersectLargeMesh,
};

///////////////////////////////////////////////////////////////////////////////
// Ray Intesect Function

bool rayIntersectDummy(
	const CollPrim &prim,
	const Transform3 &transform,
	Ray &ray,float &t )
{
	(void) prim;
	(void) transform;
	(void) ray;
	(void) t;

	return false;
}

bool rayIntersectSphere(
	const CollPrim &prim,
	const Transform3 &transform,
	Ray &ray,float &t )
{
	const Sphere &sphere = prim.getSphere();
	
	Vector3 v = ray.startPos - transform.getTranslation();

	float a = dot(ray.rayDir,ray.rayDir);
	float b = dot(v,ray.rayDir);
	float c = dot(v,v) - sphere.radius * sphere.radius;

	float d = b * b - a * c;
	
	if(d < 0.0f || fabs(a) < 0.00001f) return false;
	
	float tt = ( -b - sqrtf(d) ) / a;
	
	if(tt < 0.0f || tt > 1.0f) return false;
	
	if(tt < t) {
		t = tt;
		ray.contactPoint = ray.startPos + t * ray.rayDir;
		ray.contactNormal = normalize(ray.contactPoint-transform.getTranslation());
		ray.subData.type = SubData::SubDataNone;
		return true;
	}

	return false;
}

bool rayIntersectBox(
	const CollPrim &prim,
	const Transform3 &transform,
	Ray &ray,float &t )
{
	const Box &box = prim.getBox();
	
	// レイをBoxのローカル座標へ変換
	Transform3 transformBox = orthoInverse(transform);
	Vector3 startPosL = transformBox.getUpper3x3() * ray.startPos + transformBox.getTranslation();
	Vector3 rayDirL = transformBox.getUpper3x3() * ray.rayDir;

	// 交差判定
	float cur_t;
	Vector3 cur_nml(0.0f);
	if(rayIntersectAABB(box.half,Vector3(0.0f),startPosL,rayDirL,cur_t,cur_nml) && cur_t < t) {
		t = cur_t;
		ray.contactPoint = ray.startPos + t * ray.rayDir;
		ray.contactNormal = transform.getUpper3x3() * cur_nml;
		ray.subData.type = SubData::SubDataNone;
		return true;
	}
	
	return false;
}

bool rayIntersectCapsule(
	const CollPrim &prim,
	const Transform3 &transform,
	Ray &ray,float &t )
{
	const Capsule &capsule = prim.getCapsule();

	// レイをCapsuleのローカル座標へ変換
	Transform3 transformCapsule = orthoInverse(transform);
	Vector3 startPosL = transformCapsule.getUpper3x3() * ray.startPos + transformCapsule.getTranslation();
	Vector3 rayDirL = transformCapsule.getUpper3x3() * ray.rayDir;
	
	float radSqr = capsule.radius * capsule.radius;

	// 始点がカプセルの内側にあるか判定
	{
		float h = fabsf(startPosL[0]);
		if(h > capsule.hLength) h = capsule.hLength;
		Vector3 Px(t,0,0);
		float sqrLen = lengthSqr(startPosL-Px);
		if(sqrLen <= radSqr) return false;
	}

	// カプセルの胴体との交差判定
	do {
		Vector3 P(startPosL);
		Vector3 D(rayDirL);
		
		P[0] = 0.0f;
		D[0] = 0.0f;

		float a = dot(D,D);
		float b = dot(P,D);
		float c = dot(P,P) - radSqr;

		float d = b * b - a * c;
		
		if(d < 0.0f || fabs(a) < 0.00001f) return false;
		
		float tt = ( -b - sqrtf(d) ) / a;

		if(tt < 0.0f)
			break;
		else if(tt > 1.0f)
			return false;
		
		if(tt < t) {
			Vector3 cp = startPosL + tt * rayDirL;
			
			if(fabsf(cp[0]) <= capsule.hLength) {
				t = tt;
				ray.contactPoint = Vector3(transform * Point3(cp));
				cp[0] = 0.0f;
				ray.contactNormal = transform.getUpper3x3() * normalize(cp);
				ray.subData.type = SubData::SubDataNone;
				return true;
			}
		}
	} while(0);
	
	// カプセルの両端にある球体との交差判定
	float a = dot(rayDirL,rayDirL);
	if(fabs(a) < 0.00001f) return false;
	
	do {
		Vector3 center(capsule.hLength,0.0f,0.0f);
		Vector3 v = startPosL - center;

		float b = dot(v,rayDirL);
		float c = dot(v,v) - radSqr;

		float d = b * b - a * c;
		
		if(d < 0.0f) break;
		
		float tt = ( -b - sqrtf(d) ) / a;
		
		if(tt < 0.0f || tt > 1.0f) break;
		
		if(tt < t) {
			t = tt;
			Vector3 cp = startPosL + t * rayDirL;
			ray.contactPoint = ray.startPos + t * ray.rayDir;
			ray.contactNormal = transform.getUpper3x3() * normalize(cp-center);
			ray.subData.type = SubData::SubDataNone;
			return true;
		}
	} while(0);
	
	{
		Vector3 center(-capsule.hLength,0.0f,0.0f);
		Vector3 v = startPosL - center;

		float b = dot(v,rayDirL);
		float c = dot(v,v) - radSqr;

		float d = b * b - a * c;
		
		if(d < 0.0f) return false;
		
		float tt = ( -b - sqrtf(d) ) / a;
		
		if(tt < 0.0f || tt > 1.0f) return false;
		
		if(tt < t) {
			t = tt;
			Vector3 cp = startPosL + t * rayDirL;
			ray.contactPoint = ray.startPos + t * ray.rayDir;
			ray.contactNormal = transform.getUpper3x3() * normalize(cp-center);
			ray.subData.type = SubData::SubDataNone;
			return true;
		}
	}

	return false;
}

bool rayIntersectConvex(
	const CollPrim &prim,
	const Transform3 &transform,
	Ray &ray,float &t )
{
	ConvexMesh *mesh;


	mesh = prim.getConvexMesh();

	// レイをConvexMeshのローカル座標へ変換
	Transform3 transformConvexMesh = orthoInverse(transform);
	Vector3 rayStartL = transformConvexMesh.getUpper3x3() * ray.startPos + transformConvexMesh.getTranslation();
	Vector3 rayDirL = transformConvexMesh.getUpper3x3() * ray.rayDir;

	// レイとConvexMeshの面の交差判定
	float cur_t;
	Vector3 cur_nml(0.0f);
	bool ret = false;
	for(uint8_t f=0;f<mesh->numIndices/3;f++) {
		Vector3 facetPnts[3] = {
			mesh->verts[mesh->indices[f*3  ]],
			mesh->verts[mesh->indices[f*3+1]],
			mesh->verts[mesh->indices[f*3+2]],
		};

		if(rayIntersectTriangle(facetPnts,rayStartL,rayDirL,cur_t) && cur_t < t) {
			t = cur_t;
			ray.contactPoint = ray.startPos + t * ray.rayDir;
			ray.contactNormal = transform.getUpper3x3() * normalize(cross(facetPnts[2]-facetPnts[1],facetPnts[0]-facetPnts[1]));
			ray.subData.type = SubData::SubDataNone;
			ret = true;
		}
	}


	return ret;
}

inline
bool rayIntersectTriMesh(
	const TriMesh *mesh,
	const Vector3 &rayStartL,
	const Vector3 &rayDirL,
	uint8_t facetType,
	float &t,
	Vector3 &nml,
	SubData &subData)
{
	float cur_t;
	Vector3 cur_nml;
	bool ret = false;
	uint8_t deepestFacetId = 0;

	for(uint8_t f=0;f<mesh->numFacets;f++) {
		const MeshFacet &facet = mesh->facets[f];

		Vector3 facetPnts[3] = {
			mesh->verts[facet.vertIndices[0]],
			mesh->verts[facet.vertIndices[1]],
			mesh->verts[facet.vertIndices[2]],
		};
		
		if(facetType == FacetTypeFront && rayIntersectTriangleWithoutBackFace(facetPnts,rayStartL,rayDirL,cur_t) && cur_t < t) {
			deepestFacetId = f;
			t = cur_t;
			nml = read_Vector3(facet.normal);
			ret = true;
		}
		else if(facetType == FacetTypeBack && rayIntersectTriangleWithoutFrontFace(facetPnts,rayStartL,rayDirL,cur_t) && cur_t < t) {
			deepestFacetId = f;
			t = cur_t;
			nml = read_Vector3(facet.normal);
			ret = true;
		}
		else if(facetType == FacetTypeBoth && rayIntersectTriangle(facetPnts,rayStartL,rayDirL,cur_t) && cur_t < t) {
			deepestFacetId = f;
			t = cur_t;
			nml = read_Vector3(facet.normal);
			ret = true;
		}
	}

	if(ret) {
		// 面のローカル座標を算出
		const MeshFacet &facet = mesh->facets[deepestFacetId];
		Vector3 facetPnts[3] = {
			mesh->verts[facet.vertIndices[0]],
			mesh->verts[facet.vertIndices[1]],
			mesh->verts[facet.vertIndices[2]],
		};
		float fs,ft;
		Vector3 P = rayStartL + t * rayDirL;
		get_ST(fs,ft,facetPnts[1]-facetPnts[0],facetPnts[2]-facetPnts[0],P-facetPnts[0]);
		subData.type = SubData::SubDataFacetLocal;
		subData.setFacetLocalS(fs);
		subData.setFacetLocalT(ft);
		subData.setFacetIndex(deepestFacetId);
	}

	return ret;
}


bool rayIntersectLargeMesh(
	const CollPrim &prim,
	const Transform3 &transform,
	Ray &ray,float &t )
{
	// レイをローカル座標へ変換
	Transform3 transformMesh = orthoInverse(transform);
	Vector3 startPosL = transformMesh.getUpper3x3() * ray.startPos + transformMesh.getTranslation();
	Vector3 rayDirL = transformMesh.getUpper3x3() * ray.rayDir;

	LargeTriMesh *largeMesh = prim.getLargeMesh();

	VecInt3 s,e,aabbMinL,aabbMaxL;

	s = largeMesh->getLocalPosition(startPosL);
	e = largeMesh->getLocalPosition(startPosL+rayDirL);

	aabbMaxL = minPerElem(s,e);
	aabbMaxL = maxPerElem(s,e);

	float cur_t;
	Vector3 cur_nml(0.0f);
	bool ret = false;
	
	for(uint8_t i=0;i<largeMesh->numIslands;i++) {
		// AABBチェック
		PfxAABB16 aabbB = largeMesh->aabbList[i];
		if(aabbMaxL.getX() < XMin(aabbB) || aabbMinL.getX() > XMax(aabbB)) continue;
		if(aabbMaxL.getY() < YMin(aabbB) || aabbMinL.getY() > YMax(aabbB)) continue;
		if(aabbMaxL.getZ() < ZMin(aabbB) || aabbMinL.getZ() > ZMax(aabbB)) continue;

		Vector3 aabbMin,aabbMax;
		aabbMin = largeMesh->getWorldPosition(VecInt3((int32_t)XMin(aabbB),(int32_t)YMin(aabbB),(int32_t)ZMin(aabbB)));
		aabbMax = largeMesh->getWorldPosition(VecInt3((int32_t)XMax(aabbB),(int32_t)YMax(aabbB),(int32_t)ZMax(aabbB)));

		if( !rayIntersectAABBFast((aabbMax-aabbMin)*0.5f,(aabbMin+aabbMax)*0.5f,startPosL,rayDirL,cur_t) )
			continue;
		
		if( t <= cur_t ) continue;

		// アイランドとの交差チェック
		TriMesh *island = &largeMesh->islands[i];
		SubData subData;

		if( rayIntersectTriMesh(island,startPosL,rayDirL,ray.facetType,t,cur_nml,subData) ) {
			ray.contactPoint = ray.startPos + t * ray.rayDir;
			ray.contactNormal = transform.getUpper3x3() * cur_nml;
			ray.subData = subData;
			ray.subData.setIslandIndex(i);
			ret = true;
		}
	}

	return ret;
}


///////////////////////////////////////////////////////////////////////////////

// レイと形状との交差判定
bool rayIntersect(
	const CollObject &obj,
	const Transform3 &transform,
	Ray &ray,float &t)
{
	bool intersectFlag = false;

	PrimIterator itrPrim(obj);
	for(int p=0;p<obj.getNumPrims();p++,++itrPrim) {
		const CollPrim &prim = *itrPrim;
		Transform3 primTransform = transform * prim.getObjectRelTransform();

		if(funcTbl_rayIntersect[prim.getType()](prim,primTransform,ray,t)) {
			ray.primIdx = p;
			intersectFlag = true;
		}
	}
	
	return intersectFlag;
}

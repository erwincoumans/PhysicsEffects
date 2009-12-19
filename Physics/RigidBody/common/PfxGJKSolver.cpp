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
#include <stdio.h>
#include <string.h>
#include <float.h>

#include "PfxGJKSolver.h"

#include "Physics/Base/PerfCounter.h"
#include "Physics/Base/SimpleStack.h"
#include "Physics/RigidBody/common/CollPrim.h"
#include "Physics/RigidBody/common/intersectFunction.h"



///////////////////////////////////////////////////////////////////////////////
// Allocate Buffers

PfxGJKSolver::PfxGJKSolver(void *sA,void *sB,GetSupportVertexFunc fA,GetSupportVertexFunc fB)
{
	shapeA = sA;
	shapeB = sB;
	getSupportVertexShapeA = fA;
	getSupportVertexShapeB = fB;
	vertsP = g_vertsP;
	vertsQ = g_vertsQ;
	vertsW = g_vertsW;
	facets = g_facets;
	facetsHead = g_facetsHead;
	edges = g_edges;
}

PfxGJKSolver::~PfxGJKSolver()
{
}

///////////////////////////////////////////////////////////////////////////////
// Construct Silhouette

void PfxGJKSolver::silhouette(PfxGJKFacet *facet,int i,Vector3 w)
{
	SimpleStack<PfxGJKEdge> gs;

	gs.push(PfxGJKEdge(facet,i));

	do {
		PfxGJKEdge stk = gs.pop();
		PfxGJKFacet *ft = stk.f;

		if(ft->obsolete==0) {
			// wから見えるかどうかを判定
			if(dot(ft->normal,w-ft->closest) < 0.0f) {
				// 見えないのでエッジを登録
				PFX_ASSERT(numEdges<=MAX_FACETS);
				edges[numEdges] = stk;
				numEdges++;
			}
			else {
				ft->obsolete = 1;
				gs.push(PfxGJKEdge(ft->adj[(stk.i+2)%3],ft->j[(stk.i+2)%3]));
				gs.push(PfxGJKEdge(ft->adj[(stk.i+1)%3],ft->j[(stk.i+1)%3]));
			}
		}
	} while(PFX_UNLIKELY(!gs.isEmpty()));
}

///////////////////////////////////////////////////////////////////////////////
// Detect Penetration Depth (EPA)

float PfxGJKSolver::detectPenetrationDepth(
	const Transform3 &transformA,const Transform3 &transformB,
	Vector3 &pA,Vector3 &pB,Vector3 &normal)
{
	Matrix3 invRotA = transpose(transformA.getUpper3x3());
	Matrix3 invRotB = transpose(transformB.getUpper3x3());

	int epaIterationCount = 0;
	float distance = FLT_MAX;

	insert_bookmark(BOOKMARK_02);

	numFacets = 0;
	numFacetsHead = 0;

	// 初期状態の判定
	if(mSimplex.numVertices <= 1) {
		return distance;
	}
	else if(mSimplex.numVertices == 2) {
		Vector3 v0 = mSimplex.W[0];
		Vector3 v1 = mSimplex.W[1];
		float t = -dot(v0,v1)/(dot(v1,v1)-dot(v0,v1));
		Vector3 tmp = v0 + t * (v1-v0);
		Vector3 dir = normalize(v1-v0);
		Matrix3 rot = Matrix3::rotation(2.0943951023932f,dir);//120 deg
		int axis;
		if(dir[0] < dir[1]) {
			if(dir[0] < dir[2]) {
				axis = 0;
			}
			else {
				axis = 2;
			}
		}
		else {
			if(dir[1] < dir[2]) {
				axis = 1;
			}
			else {
				axis = 2;
			}
		}
        Vector3 vec(0.0f);
		vec[axis] = 1.0f;

		Vector3 aux[3];
		aux[0] = cross(dir,vec);
		aux[1] = rot * aux[0];
		aux[2] = rot * aux[1];

		Vector3 p[3],q[3],w[3];

		for(int i=0;i<3;i++) {
			Vector3 pInA,qInB;
			getSupportVertexShapeA(shapeA,invRotA * aux[i],pInA);
			getSupportVertexShapeB(shapeB,invRotB * (-aux[i]),qInB);
			p[i] = transformA.getTranslation() + transformA.getUpper3x3() * pInA;
			q[i] = transformB.getTranslation() + transformB.getUpper3x3() * qInB;
			w[i] = p[i] - q[i];
			vertsP[i] = p[i];
			vertsQ[i] = q[i];
			vertsW[i] = w[i];
		}

		if(originInTetrahedron(w[0],w[1],w[2],v0)) {
			vertsP[3] = mSimplex.P[0];
			vertsQ[3] = mSimplex.Q[0];
			vertsW[3] = mSimplex.W[0];
			numVerts = 4;
		}
		else if(originInTetrahedron(w[0],w[1],w[2],v1)){
			vertsP[3] = mSimplex.P[1];
			vertsQ[3] = mSimplex.Q[1];
			vertsW[3] = mSimplex.W[1];
			numVerts = 4;
		}
		else {
			return distance;
		}
	}
	else if(mSimplex.numVertices == 3) {
		numVerts = 3;
		for(int i=0;i<numVerts;i++) {
			vertsP[i] = mSimplex.P[i];
			vertsQ[i] = mSimplex.Q[i];
			vertsW[i] = mSimplex.W[i];
		}

		Vector3 p[2],q[2],w[2];

		{
			Vector3 v = cross(vertsW[2]-vertsW[0],vertsW[1]-vertsW[0]);
			Vector3 pInA,qInB;
			getSupportVertexShapeA(shapeA,invRotA * v,pInA);
			getSupportVertexShapeB(shapeB,invRotB * (-v),qInB);
			p[0] = transformA.getTranslation() + transformA.getUpper3x3() * pInA;
			q[0] = transformB.getTranslation() + transformB.getUpper3x3() * qInB;
			w[0] = p[0] - q[0];
			getSupportVertexShapeA(shapeA,invRotA * (-v),pInA);
			getSupportVertexShapeB(shapeB,invRotB * v,qInB);
			p[1] = transformA.getTranslation() + transformA.getUpper3x3() * pInA;
			q[1] = transformB.getTranslation() + transformB.getUpper3x3() * qInB;
			w[1] = p[1] - q[1];
		}

		if(originInTetrahedron(vertsW[0],vertsW[1],vertsW[2],w[0])) {
			vertsP[3] = p[0];
			vertsQ[3] = q[0];
			vertsW[3] = w[0];
			numVerts = 4;
		}
		else if(originInTetrahedron(vertsW[0],vertsW[1],vertsW[2],w[1])){
			vertsP[3] = p[1];
			vertsQ[3] = q[1];
			vertsW[3] = w[1];
			numVerts = 4;
		}
		else {
			return distance;
		}
	}
	else {
		numVerts = 4;
		for(int i=0;i<numVerts;i++) {
			vertsP[i] = mSimplex.P[i];
			vertsQ[i] = mSimplex.Q[i];
			vertsW[i] = mSimplex.W[i];
		}
	}

	PFX_ASSERT(numVerts == 4);

	// 原点が単体の内部にあるかどうかを判定
	if(PFX_UNLIKELY(!originInTetrahedron(vertsW[0],vertsW[1],vertsW[2],vertsW[3]))) {
		return distance;
	}

	// 面の向きをチェック
	if(dot(-vertsW[0],cross(vertsW[2]-vertsW[0],vertsW[1]-vertsW[0])) > 0.0f) {
		Vector3 vertsP1,vertsQ1,vertsW1;
		Vector3 vertsP3,vertsQ3,vertsW3;
		vertsQ1=vertsQ[1];vertsW1=vertsW[1];vertsP1=vertsP[1];
		vertsQ3=vertsQ[3];vertsW3=vertsW[3];vertsP3=vertsP[3];
		vertsQ[1]=vertsQ3;vertsW[1]=vertsW3;vertsP[1]=vertsP3;
		vertsQ[3]=vertsQ1;vertsW[3]=vertsW1;vertsP[3]=vertsP1;
	}

	{
		PfxGJKFacet *f0 = addFacet(0,1,2);
		PfxGJKFacet *f1 = addFacet(0,3,1);
		PfxGJKFacet *f2 = addFacet(0,2,3);
		PfxGJKFacet *f3 = addFacet(1,3,2);

		if(PFX_UNLIKELY(!f0 || !f1 || !f2 || !f3)) return distance;

		linkFacets(f0,0,f1,2);
		linkFacets(f0,1,f3,2);
		linkFacets(f0,2,f2,0);
		linkFacets(f1,0,f2,2);
		linkFacets(f1,1,f3,0);
		linkFacets(f2,1,f3,1);
	}

	// 探索
	PfxGJKFacet *facetMin = NULL;

	do {
		insert_bookmark(BOOKMARK_03);

		// 原点から一番近い点を算出し、そのベクトルと支点を返す
		int minFacetIdx = 0;
		{
			float minDistSqr = FLT_MAX;
			for(int i=0;i<numFacetsHead;i++) {
				if(facetsHead[i]->obsolete==0 && facetsHead[i]->distSqr < minDistSqr) {
					minDistSqr = facetsHead[i]->distSqr;
					facetMin = facetsHead[i];
					minFacetIdx = i;
				}
			}
		}

		insert_bookmark(BOOKMARK_03);
		insert_bookmark(BOOKMARK_04);

		// リストからはずす
		facetsHead[minFacetIdx] = facetsHead[--numFacetsHead];

		Vector3 pInA(0.0f),qInB(0.0f);
		getSupportVertexShapeA(shapeA,invRotA * facetMin->normal,pInA);
		getSupportVertexShapeB(shapeB,invRotB * (-facetMin->normal),qInB);
		Vector3 p = transformA.getTranslation() + transformA.getUpper3x3() * pInA;
		Vector3 q = transformB.getTranslation() + transformB.getUpper3x3() * qInB;
		Vector3 w = p - q;
		Vector3 v = facetMin->closest;

		// 最近接点チェック
		float l0 = length(v);
		float l1 = dot(facetMin->normal,w);

		if((l1 - l0) < PFX_GJK_EPSILON) {
			break;
		}

		insert_bookmark(BOOKMARK_04);
		insert_bookmark(BOOKMARK_05);

		// 求めた点を追加して面を分割
		{
			PFX_ASSERT(numVerts<MAX_VERTS);
			int vId = numVerts++;
			vertsP[vId] = p;
			vertsQ[vId] = q;
			vertsW[vId] = w;

				facetMin->obsolete = 1;
			numEdges = 0;

			silhouette(facetMin->adj[0],facetMin->j[0],w);
			silhouette(facetMin->adj[1],facetMin->j[1],w);
			silhouette(facetMin->adj[2],facetMin->j[2],w);

			if(PFX_UNLIKELY(numEdges == 0)) break;

			bool edgeCheck = true;
			PfxGJKFacet *firstFacet,*lastFacet;
			{
				PfxGJKEdge &edge = edges[0];
				int v0 = edge.f->v[(edge.i+1)%3];
				int v1 = edge.f->v[edge.i];
				firstFacet = addFacet(v0,v1,vId);
				if(PFX_UNLIKELY(!firstFacet)) {
					edgeCheck = false;
					break;
				}
				linkFacets(edge.f,edge.i,firstFacet,0);
				lastFacet = firstFacet;
			}

			if(PFX_UNLIKELY(!edgeCheck)) break;

			for(int e=1;e<numEdges;e++) {
				PfxGJKEdge &edge = edges[e];
				int v0 = edge.f->v[(edge.i+1)%3];
				int v1 = edge.f->v[edge.i];
				PfxGJKFacet *f = addFacet(v0,v1,vId);
				if(PFX_UNLIKELY(!f)) {edgeCheck=false;break;}
				linkFacets(edge.f,edge.i,f,0);
				linkFacets(f,2,lastFacet,1);
				lastFacet = f;
			}
			if(PFX_UNLIKELY(!edgeCheck)) break;

			linkFacets(lastFacet,1,firstFacet,2);
		}

		insert_bookmark(BOOKMARK_05);

		epaIterationCount++;
		if(PFX_UNLIKELY(epaIterationCount > PFX_EPA_ITERATION_MAX || numFacetsHead == 0)) {
			break;
		}
	} while(1);

	// 衝突点計算
	int v1 = facetMin->v[0];
	int v2 = facetMin->v[1];
	int v3 = facetMin->v[2];

	Vector3 p0 = vertsW[v2]-vertsW[v1];
	Vector3 p1 = vertsW[v3]-vertsW[v1];
	Vector3 p2 = facetMin->closest-vertsW[v1];

	Vector3 v = cross( p0, p1 );
	Vector3 crS = cross( v, p0 );
	Vector3 crT = cross( v, p1 );
	float d0 = dot( crT, p0 );
	float d1 = dot( crS, p1 );

	if(fabsf(d0) < PFX_GJK_EPSILON || fabsf(d1) < PFX_GJK_EPSILON) return distance;

	float lamda1 = dot( crT, p2 ) / d0;
	float lamda2 = dot( crS, p2 ) / d1;

	pA = vertsP[v1] + lamda1*(vertsP[v2]-vertsP[v1]) + lamda2*(vertsP[v3]-vertsP[v1]);
	pB = vertsQ[v1] + lamda1*(vertsQ[v2]-vertsQ[v1]) + lamda2*(vertsQ[v3]-vertsQ[v1]);

	float lenSqr = lengthSqr(pB-pA);
	normal = normalize(pB-pA);

	insert_bookmark(BOOKMARK_02);

	return -sqrtf(lenSqr);
}

///////////////////////////////////////////////////////////////////////////////
// GJK

float PfxGJKSolver::collide( Vector3& normal, Point3 &pointA, Point3 &pointB,
						const Transform3 & transformA,
						const Transform3 & transformB,
						float distanceThreshold)
{
	(void) distanceThreshold;

	int gjkIterationCount = 0;

	mSimplex.reset();

	Transform3 cTransformA = transformA;
	Transform3 cTransformB = transformB;
	Matrix3 invRotA = transpose(cTransformA.getUpper3x3());
	Matrix3 invRotB = transpose(cTransformB.getUpper3x3());

	Vector3 offset = (cTransformA.getTranslation() + cTransformB.getTranslation())*0.5f;
	cTransformA.setTranslation(cTransformA.getTranslation()-offset);
	cTransformB.setTranslation(cTransformB.getTranslation()-offset);

	Vector3 separatingAxis(-cTransformA.getTranslation());
	if(lengthSqr(separatingAxis) < 0.000001f) separatingAxis = Vector3(1,0,0);
	float squaredDistance = FLT_MAX;
	float delta = 0.0f;
	float distance = FLT_MAX;

	insert_bookmark(BOOKMARK_01);

	for(;;) {
		// サポート頂点の取得
		Vector3 pInA,qInB;

		getSupportVertexShapeA(shapeA,invRotA * (-separatingAxis),pInA);
		getSupportVertexShapeB(shapeB,invRotB * separatingAxis,qInB);

		Vector3 p = cTransformA.getTranslation() + cTransformA.getUpper3x3() * pInA;
		Vector3 q = cTransformB.getTranslation() + cTransformB.getUpper3x3() * qInB;
		Vector3 w = p - q;

		delta = dot(separatingAxis,w);

		// 早期終了チェック
		if(PFX_UNLIKELY(delta > 0.0f)) {
			normal = separatingAxis;
			return distance;
		}

		if(PFX_UNLIKELY(mSimplex.inSimplex(w))) {
			break;
		}

		float f0 = squaredDistance - delta;
		float f1 = squaredDistance * PFX_GJK_EPSILON;

		if (PFX_UNLIKELY(f0 <= f1)) {
			break;
		}

		// 頂点を単体に追加
		mSimplex.addVertex(w,p,q);
		
		// 原点と単体の最近接点を求め、分離軸を返す
		if(PFX_UNLIKELY(!mSimplex.closest(separatingAxis))) {
			normal = separatingAxis;
			return distance;
		}

		squaredDistance = lengthSqr(separatingAxis);

		if(PFX_UNLIKELY(gjkIterationCount >= PFX_GJK_ITERATION_MAX || mSimplex.fullSimplex())) {
			break;
		}

		gjkIterationCount++;
	}
	insert_bookmark(BOOKMARK_01);

	Vector3 pA(0.0f),pB(0.0f);

	// ２つのConvexは交差しているので、接触点を探索する
	float dist = detectPenetrationDepth(cTransformA,cTransformB,pA,pB,normal);

	//マージン考慮
	if(dist < 0.0f) {
		pA += normal * PFX_GJK_MARGIN;
		pB -= normal * PFX_GJK_MARGIN;
		dist = dot(normal,pA-pB);
		pointA = orthoInverse(transformA)*Point3(pA+offset);
		pointB = orthoInverse(transformB)*Point3(pB+offset);
	}

	return dist;
}

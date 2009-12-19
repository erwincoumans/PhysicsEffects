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

#ifndef __PFX_GJK_SOLVER_H__
#define __PFX_GJK_SOLVER_H__

#include <float.h>

#include "Physics/Base/PhysicsCommon.h"
#include "PfxSimplexSolver.h"

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/SimdFunc.h"

#define PFX_GJK_EPSILON			1e-04f
#define PFX_GJK_MARGIN			0.05f
#define PFX_GJK_ITERATION_MAX	10
#define PFX_EPA_ITERATION_MAX	10

/*
	Memo
	pool 13280
	stack 5232 sizeof(PfxGJKSolver) 13856
	*/

///////////////////////////////////////////////////////////////////////////////
// Support Function

typedef void (*GetSupportVertexFunc)(void *shape,Vector3 seperatingAxis,Vector3 &supportVertex);

///////////////////////////////////////////////////////////////////////////////
// GJK

class PfxSimplexSolver;

class PfxGJKSolver
{
private:
	PfxGJKSolver() {}
	
	static const int   MAX_VERTS = 128;
	static const int   MAX_EDGES = 128;
	static const int   MAX_FACETS = 64;
	
	PfxSimplexSolver mSimplex;
	
	// 面
	struct PfxGJKFacet {
		Vector3 normal;			// 面の法線
		Vector3 closest;		// 原点からの最短ベクトル
		uint32_t obsolete;		// 廃棄面判定
		float distSqr;			// 原点からの距離
		int v[3];				// 頂点
		int j[3];				// 隣接面から見たIndex
		PfxGJKFacet *adj[3];	// 隣接面
	};
	
	// エッジ
	struct PfxGJKEdge {
		PfxGJKFacet *f;
		int i;
		PfxGJKEdge() {}
		PfxGJKEdge(PfxGJKFacet *f_,int i_)
		{
			f = f_;
			i= i_;
		}
	};
	
	#ifndef __SPU__
	Vector3 g_vertsP[MAX_VERTS];
	Vector3 g_vertsQ[MAX_VERTS];
	Vector3 g_vertsW[MAX_VERTS];
	PfxGJKFacet g_facets[MAX_FACETS];
	PfxGJKFacet *g_facetsHead[MAX_FACETS];
	PfxGJKEdge  g_edges[MAX_EDGES];
	#endif

	Vector3 *vertsP __attribute__ ((aligned(16)));
	Vector3 *vertsQ __attribute__ ((aligned(16)));
	Vector3 *vertsW __attribute__ ((aligned(16)));
	PfxGJKFacet *facets __attribute__ ((aligned(16)));
	PfxGJKFacet **facetsHead __attribute__ ((aligned(16)));
	PfxGJKEdge  *edges __attribute__ ((aligned(16)));
	
	int numVerts;
	int numEdges;
	int numFacets;
	int numFacetsHead;
	
	inline PfxGJKFacet *addFacet(int v1,int v2,int v3);

	inline void linkFacets(PfxGJKFacet *f1,int e1,PfxGJKFacet *f2,int e2);
	void silhouette(PfxGJKFacet *facet,int i,Vector3 w);

	inline bool originInTetrahedron(const Vector3& p0,const Vector3& p1,const Vector3& p2,const Vector3& p3);

	float detectPenetrationDepth(
		const Transform3 &transformA,const Transform3 &transformB,
		Vector3 &pA,Vector3 &pB,Vector3 &normal);

	void *shapeA;
	void *shapeB;
	GetSupportVertexFunc getSupportVertexShapeA;
	GetSupportVertexFunc getSupportVertexShapeB;

public:
	PfxGJKSolver(void *sA,void *sB,GetSupportVertexFunc fA,GetSupportVertexFunc fB);
	~PfxGJKSolver();
	
	float collide( Vector3& normal, Point3 &pointA, Point3 &pointB,
					const Transform3 & transformA,
					const Transform3 & transformB,
					float distanceThreshold = FLT_MAX);
};

inline
PfxGJKSolver::PfxGJKFacet *PfxGJKSolver::addFacet(int v1,int v2,int v3)
{
	if(PFX_UNLIKELY(numFacets == MAX_FACETS))
		return NULL;

	PfxGJKFacet &facet = facets[numFacets];
	Vector3 V1 = vertsW[v1];
	Vector3 V2 = vertsW[v2];
	Vector3 V3 = vertsW[v3];
	facet.obsolete = 0;
	facet.v[0] = v1;
	facet.v[1] = v2;
	facet.v[2] = v3;

	Vector3 normal = cross(V3-V1,V2-V1);
	
	float l = lengthSqr(normal);

	if(l < PFX_GJK_EPSILON * PFX_GJK_EPSILON) {
		return NULL;
	}

	normal /= sqrtf(l);
	facet.closest = dot(V1,normal)*normal;
	facet.normal =normal;

	facet.distSqr = lengthSqr(facet.closest);

	facetsHead[numFacetsHead++] = &facet;
	numFacets++;

	return &facet;
}

inline
void PfxGJKSolver::linkFacets(PfxGJKFacet *f1,int e1,PfxGJKFacet *f2,int e2)
{
	f1->adj[e1] = f2;
	f2->adj[e2] = f1;
	f1->j[e1] = e2;
	f2->j[e2] = e1;
}

inline
bool PfxGJKSolver::originInTetrahedron(const Vector3& p0,const Vector3& p1,const Vector3& p2,const Vector3& p3)
{
    Vector3 n0 = cross((p1-p0),(p2-p0));
    Vector3 n1 = cross((p2-p1),(p3-p1));
    Vector3 n2 = cross((p3-p2),(p0-p2));
    Vector3 n3 = cross((p0-p3),(p1-p3));
    
    return 
		dot(n0,p0) * dot(n0,p3-p0) < 0.0f &&
		dot(n1,p1) * dot(n1,p0-p1) < 0.0f &&
		dot(n2,p2) * dot(n2,p1-p2) < 0.0f &&
		dot(n3,p3) * dot(n3,p2-p3) < 0.0f;
}

#endif /* __PFX_GJK_SOLVER_H__ */

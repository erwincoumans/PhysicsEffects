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
#include "PfxGJKSolver.h"
#include "PfxGJKSupportFunc.h"
#include "Physics/Base/SimdFunc.h"
#include "Physics/RigidBody/common/TriMesh.h"
#include "Physics/RigidBody/common/Box.h"
#include "Physics/RigidBody/common/Capsule.h"
#include "Physics/RigidBody/common/Sphere.h"

///////////////////////////////////////////////////////////////////////////////
// Support Function

void getSupportVertexTriangle(void *shape,Vector3 seperatingAxis,Vector3 &supportVertex)
{
	Vector3 *vtx = (Vector3*)shape;
	
	float d0 = dot(vtx[0],seperatingAxis);
	float d1 = dot(vtx[1],seperatingAxis);
	float d2 = dot(vtx[2],seperatingAxis);

	int reti = 2;
	
	if(d0 > d1 && d0 > d2) {
		reti = 0;
	}
	else if(d1 > d2) {
		reti = 1;
	}

	supportVertex = vtx[reti] + PFX_GJK_MARGIN * normalize(seperatingAxis);
}

void getSupportVertexTriangleWithThickness(void *shape,Vector3 seperatingAxis,Vector3 &supportVertex)
{
	Vector3 *vtx = (Vector3*)shape;
	
	float d[6];
	d[0] = dot(vtx[0],seperatingAxis);
	d[1] = dot(vtx[1],seperatingAxis);
	d[2] = dot(vtx[2],seperatingAxis);
	d[3] = dot(vtx[3],seperatingAxis);
	d[4] = dot(vtx[4],seperatingAxis);
	d[5] = dot(vtx[5],seperatingAxis);
	
	int reti = 0;
	for(int i=1;i<6;i++) {
		if(d[reti] < d[i]) {
			reti = i;
		}
	}

	supportVertex = vtx[reti] + PFX_GJK_MARGIN * normalize(seperatingAxis);
}

void getSupportVertexConvex(void *shape,Vector3 seperatingAxis,Vector3 &supportVertex)
{
	ConvexMesh *mesh = (ConvexMesh*)shape;
	int reti = 0;
	float dmax = dot(mesh->verts[0],seperatingAxis);
	for(int i=1;i<mesh->numVerts;i++) {
		float d = dot(mesh->verts[i],seperatingAxis);
		if(d > dmax) {
			dmax =d;
			reti = i;
		}
	}
	supportVertex = mesh->verts[reti] + PFX_GJK_MARGIN * normalize(seperatingAxis);
}

void getSupportVertexBox(void *shape,Vector3 seperatingAxis,Vector3 &supportVertex)
{
	Box *box = (Box*)shape;
	Vector3 boxHalf = box->half + Vector3(PFX_GJK_MARGIN);
	supportVertex[0] = seperatingAxis[0]>0.0f?boxHalf[0]:-boxHalf[0];
	supportVertex[1] = seperatingAxis[1]>0.0f?boxHalf[1]:-boxHalf[1];
	supportVertex[2] = seperatingAxis[2]>0.0f?boxHalf[2]:-boxHalf[2];
}

void getSupportVertexCapsule(void *shape,Vector3 seperatingAxis,Vector3 &supportVertex)
{
	Capsule *capsule = (Capsule*)shape;
	Vector3 u(1.0f,0.0f,0.0f);

	float udotv = dot(seperatingAxis,u);
	Vector3 dir = u * (udotv > 0.0f ? capsule->hLength : -capsule->hLength);
	supportVertex = dir + normalize(seperatingAxis) * (capsule->radius + PFX_GJK_MARGIN);
}

void getSupportVertexSphere(void *shape,Vector3 seperatingAxis,Vector3 &supportVertex)
{
	Sphere *sphere = (Sphere*)shape;
	supportVertex = normalize(seperatingAxis) * (sphere->radius + PFX_GJK_MARGIN);
}

/*
void getSupportVertexCylinder(void *shape,Vector3 seperatingAxis,Vector3 &supportVertex)
{
	Cylinder *cylinder = (Cylinder*)shape;
	Vector3 u(1.0f,0.0f,0.0f);

	float udotv = dot(seperatingAxis,u);
	Vector3 dir = u * (udotv > 0.0f ? capsule->hLength : -capsule->hLength);
	supportVertex = dir + normalize(seperatingAxis) * PFX_GJK_MARGIN;
}
*/

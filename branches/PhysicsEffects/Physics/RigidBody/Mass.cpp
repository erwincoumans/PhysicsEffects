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

#include "Mass.h"
#include "Physics/Base/PhysicsCommon.h"

///////////////////////////////////////////////////////////////////////////////
// Box

float calcMassBox(float density,const Vector3 &size)
{
	return density * size[0] * size[1] * size[2] * 8;
}

void calcInertiaBox(const Vector3 &size,float mass,Matrix3 &inertia)
{
	const float ratio = 1.2f;
	float sqrwidth  = size[0] * 2.0f * ratio;
	float sqrheight = size[1] * 2.0f * ratio;
	float sqrdepth  = size[2] * 2.0f * ratio;
	sqrwidth *= sqrwidth;
	sqrheight *= sqrheight;
	sqrdepth *= sqrdepth;
	inertia = Matrix3::identity();
	inertia[0][0] = (mass*(sqrheight+sqrdepth))/12.0f;
	inertia[1][1] = (mass*(sqrwidth+sqrdepth))/12.0f;
	inertia[2][2] = (mass*(sqrwidth+sqrheight))/12.0f;
}

///////////////////////////////////////////////////////////////////////////////
// Sphere

float calcMassSphere(float density,float radius)
{
	return (4.0f/3.0f) * PFX_PI * radius * radius * radius * density;
}

void calcInertiaSphere(float radius,float mass,Matrix3 &inertia)
{
	const float ratio = 1.2f;
	inertia = Matrix3::identity();
	inertia[0][0] = inertia[1][1] = inertia[2][2] = 0.4f * mass * radius * radius * ratio * ratio;
}

///////////////////////////////////////////////////////////////////////////////
// Cylinder

float calcMassCylinder(float density,float radius,float halfHeight)
{
	return PFX_PI * radius * radius * 2.0f * halfHeight * density;
}

void calcInertiaCylinder(float radius,float halfHeight,float mass,Matrix3 &inertia,int axis)
{
	inertia = Matrix3::identity();
	inertia[0][0] = inertia[1][1] = inertia[2][2] = 0.25f * mass * radius * radius + 0.33f * mass * halfHeight * halfHeight; 
	inertia[axis][axis] = 0.5f * mass * radius * radius;
}

void calcInertiaCylinderX(float radius,float halfHeight,float mass,Matrix3 &inertia)
{
	calcInertiaCylinder(radius,halfHeight,mass,inertia,0);
}

void calcInertiaCylinderY(float radius,float halfHeight,float mass,Matrix3 &inertia)
{
	calcInertiaCylinder(radius,halfHeight,mass,inertia,1);
}

void calcInertiaCylinderZ(float radius,float halfHeight,float mass,Matrix3 &inertia)
{
	calcInertiaCylinder(radius,halfHeight,mass,inertia,2);
}

///////////////////////////////////////////////////////////////////////////////
// TriMesh

// 三角錐の体積
inline float calcTrigonalVolume(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3)
{
	return 	dot(cross(p1,p2),p3) / 6.0f;
}

// 三角錐の慣性テンソル
Matrix3 calcTrigonalInertia(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3,float mass)
{
	Matrix3 retI;
	
	retI[0][0] = (p1[1] * p1[1] + p1[2] * p1[2]
				+ p2[1] * p2[1] + p2[2] * p2[2]
				+ p3[1] * p3[1] + p3[2] * p3[2]
				+ p1[1] * p2[1] + p1[2] * p2[2]
				+ p2[1] * p3[1] + p2[2] * p3[2]
				+ p3[1] * p1[1] + p3[2] * p1[2]) * mass * 0.1f;

	retI[1][0] = retI[0][1] = (-2.0f * p1[0] * p1[1]
				-p1[0] * p2[1]
				-p1[0] * p3[1]
				-p2[0] * p1[1]
				-2.0f * p2[0] * p2[1]
				-p2[0] * p3[1]
				-p3[0] * p1[1]
				-p3[0] * p2[1]
				-2.0f * p3[0] * p3[1]) * mass * 0.05f;
	retI[2][0] = retI[0][2] = (-2.0f * p1[0] * p1[2]
				-p1[0] * p2[2]
				-p1[0] * p3[2]
				-p2[0] * p1[2]
				-2.0f * p2[0] * p2[2]
				-p2[0] * p3[2]
				-p3[0] * p1[2]
				-p3[0] * p2[2]
				-2.0f * p3[0] * p3[2]) * mass * 0.05f;
	retI[1][1] = (p1[2] * p1[2] + p1[0] * p1[0]
				+ p2[2] * p2[2] + p2[0] * p2[0]
				+ p3[2] * p3[2] + p3[0] * p3[0]
				+ p1[2] * p2[2] + p1[0] * p2[0]
				+ p2[2] * p3[2] + p2[0] * p3[0]
				+ p3[2] * p1[2] + p3[0] * p1[0]) * mass * 0.1f;
	retI[2][1] = retI[1][2] = (-2.0f * p1[0] * p1[1]
				-p1[0] * p2[1]
				-p1[0] * p3[1]
				-p2[0] * p1[1]
				-2.0f * p2[0] * p2[1]
				-p2[0] * p3[1]
				-p3[0] * p1[1]
				-p3[0] * p2[1]
				-2.0f * p3[0] * p3[1]) * mass * 0.05f;
	retI[2][2] = (p1[0] * p1[0] + p1[1] * p1[1]
				+ p2[0] * p2[0] + p2[1] * p2[1]
				+ p3[0] * p3[0] + p3[1] * p3[1]
				+ p1[0] * p2[0] + p1[1] * p2[1]
				+ p2[0] * p3[0] + p2[1] * p3[1]
				+ p3[0] * p1[0] + p3[1] * p1[1]) * mass * 0.1f;
	
	return retI;
}

float calcVolumeMesh(const TriMesh &mesh)
{
	float allVolume = 0.0f;
	for(uint32_t f=0;f<mesh.numFacets;f++) {
		const MeshFacet &facet = mesh.facets[f];
		
		float tV = calcTrigonalVolume(
			mesh.verts[facet.vertIndices[0]],
			mesh.verts[facet.vertIndices[1]],
			mesh.verts[facet.vertIndices[2]]);
		
		allVolume += tV;
	}
	
	return allVolume;
}

float calcMassMesh(float density,const TriMesh &mesh)
{
	return calcVolumeMesh(mesh) * density;
}

void calcInertiaMesh(const TriMesh &mesh,float mass,Matrix3 &inertia)
{
	float allVolume = calcVolumeMesh(mesh);
	Matrix3 allInertia(0.0f);

	for(uint32_t f=0;f<mesh.numFacets;f++) {
		const MeshFacet &facet = mesh.facets[f];

		float tV = calcTrigonalVolume(
			mesh.verts[facet.vertIndices[0]],
			mesh.verts[facet.vertIndices[1]],
			mesh.verts[facet.vertIndices[2]]);
		
		Matrix3 tI = calcTrigonalInertia(
			mesh.verts[facet.vertIndices[0]],
			mesh.verts[facet.vertIndices[1]],
			mesh.verts[facet.vertIndices[2]],
			mass * (tV / allVolume));
		
		allInertia += tI;
	}
	
	inertia = allInertia;
}

float calcVolumeMesh(const float *verts,int numVerts,const unsigned short *indices,int numIndices)
{
	(void) numVerts;
	float allVolume = 0.0f;
	for(uint32_t f=0;f<(uint32_t)numIndices/3;f++) {
		float tV = calcTrigonalVolume(
			Vector3(verts[indices[f*3  ]*3  ],verts[indices[f*3  ]*3+1],verts[indices[f*3  ]*3+2]),
			Vector3(verts[indices[f*3+1]*3  ],verts[indices[f*3+1]*3+1],verts[indices[f*3+1]*3+2]),
			Vector3(verts[indices[f*3+2]*3  ],verts[indices[f*3+2]*3+1],verts[indices[f*3+2]*3+2])
			);
		allVolume += tV;
	}
	
	return allVolume;
}

float calcVolumeMesh(const ConvexMesh &mesh)
{
	float allVolume = 0.0f;
	for(uint32_t f=0;f<mesh.numIndices/3;f++) {
		float tV = calcTrigonalVolume(
			mesh.verts[mesh.indices[f*3  ]],
			mesh.verts[mesh.indices[f*3+1]],
			mesh.verts[mesh.indices[f*3+2]]);
		allVolume += tV;
	}
	
	return allVolume;
}

float calcMassMesh(float density,const ConvexMesh &mesh)
{
	return calcVolumeMesh(mesh) * density;
}

void calcInertiaMesh(const ConvexMesh &mesh,float mass,Matrix3 &inertia)
{
	float allVolume = calcVolumeMesh(mesh);
	Matrix3 allInertia(0.0f);

	for(uint32_t f=0;f<mesh.numIndices/3;f++) {
		float tV = calcTrigonalVolume(
			mesh.verts[mesh.indices[f*3  ]],
			mesh.verts[mesh.indices[f*3+1]],
			mesh.verts[mesh.indices[f*3+2]]);
		
		Matrix3 tI = calcTrigonalInertia(
			mesh.verts[mesh.indices[f*3  ]],
			mesh.verts[mesh.indices[f*3+1]],
			mesh.verts[mesh.indices[f*3+2]],
			mass * (tV / allVolume));
		
		allInertia += tI;
	}
	
	inertia = allInertia;
}

float calcMassMesh(float density,const float *verts,int numVerts,const unsigned short *indices,int numIndices)
{
	return calcVolumeMesh(verts,numVerts,indices,numIndices) * density;
}

void calcInertiaMesh(const float *verts,int numVerts,const unsigned short *indices,int numIndices,float mass,Matrix3 &inertia)
{
	float allVolume = calcVolumeMesh(verts,numVerts,indices,numIndices);
	Matrix3 allInertia(0.0f);

	for(uint32_t f=0;f<(uint32_t)numIndices/3;f++) {
		float tV = calcTrigonalVolume(
			Vector3(verts[indices[f*3  ]*3  ],verts[indices[f*3  ]*3+1],verts[indices[f*3  ]*3+2]),
			Vector3(verts[indices[f*3+1]*3  ],verts[indices[f*3+1]*3+1],verts[indices[f*3+1]*3+2]),
			Vector3(verts[indices[f*3+2]*3  ],verts[indices[f*3+2]*3+1],verts[indices[f*3+2]*3+2])
			);
		
		Matrix3 tI = calcTrigonalInertia(
			Vector3(verts[indices[f*3  ]*3  ],verts[indices[f*3  ]*3+1],verts[indices[f*3  ]*3+2]),
			Vector3(verts[indices[f*3+1]*3  ],verts[indices[f*3+1]*3+1],verts[indices[f*3+1]*3+2]),
			Vector3(verts[indices[f*3+2]*3  ],verts[indices[f*3+2]*3+1],verts[indices[f*3+2]*3+2]),
			mass * (tV / allVolume));
		
		allInertia += tI;
	}
	
	inertia = allInertia;
}

///////////////////////////////////////////////////////////////////////////////

void massTranslate(float mass,Matrix3 &inertia,const Vector3 &translation)
{
	Matrix3 m = crossMatrix(translation);
	inertia = inertia + mass * (-m*m);
}

void massRotate(Matrix3 &inertia,const Matrix3 &rotate)
{
	inertia = rotate * inertia * transpose(rotate);
}

void massMerge(float mass1,Matrix3 &inertia1,Vector3 &p1,float mass2,const Matrix3 &inertia2,const Vector3 &p2)
{
	p1 = (mass1*p1 + mass2*p2) / (mass1 + mass2);
	mass1 = mass1 + mass2;
	inertia1 = inertia1 + inertia2;
}

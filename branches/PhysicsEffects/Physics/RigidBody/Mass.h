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

///////////////////////////////////////////////////////////////////////////////
// 質量・慣性テンソル

#ifndef __MASS_H__
#define __MASS_H__

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/RigidBody/common/TriMesh.h"

///////////////////////////////////////////////////////////////////////////////
// 形状ごとの質量計算

// Box
float calcMassBox(float density,const Vector3 &size);
void calcInertiaBox(const Vector3 &size,float mass,Matrix3 &inertia);

// Sphere
float calcMassSphere(float density,float radius);
void calcInertiaSphere(float radius,float mass,Matrix3 &inertia);

// Cylinder
float calcMassCylinder(float density,float radius,float halfHeight);
void calcInertiaCylinderX(float radius,float halfHeight,float mass,Matrix3 &inertia);
void calcInertiaCylinderY(float radius,float halfHeight,float mass,Matrix3 &inertia);
void calcInertiaCylinderZ(float radius,float halfHeight,float mass,Matrix3 &inertia);

// TriMesh
float calcMassMesh(float density,const TriMesh &mesh);
void calcInertiaMesh(const TriMesh &mesh,float mass,Matrix3 &inertia);

// ConvexMesh
float calcMassMesh(float density,const ConvexMesh &mesh);
void calcInertiaMesh(const ConvexMesh &mesh,float mass,Matrix3 &inertia);

// Mesh
float calcMassMesh(float density,const float *verts,int numVerts,const unsigned short *indices);
void calcInertiaMesh(const float *verts,int numVerts,const unsigned short *indices,int numIndices,float mass,Matrix3 &inertia);

///////////////////////////////////////////////////////////////////////////////
// 質量の移動・回転・合成

// 質量の移動
void massTranslate(float mass,Matrix3 &inertia,const Vector3 &translation);

// 質量の回転
void massRotate(Matrix3 &inertia,const Matrix3 &rotate);

// 質量の合成（mass1,inertia1に合成した質量が入る）
void massMerge(float mass1,Matrix3 &inertia1,Vector3 &p1,float mass2,const Matrix3 &inertia2,const Vector3 &p2);

#endif

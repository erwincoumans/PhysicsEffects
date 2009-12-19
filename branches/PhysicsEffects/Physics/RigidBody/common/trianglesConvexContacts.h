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

#ifndef __TRIANGLESCONVEX_CONTACTS_H__
#define __TRIANGLESCONVEX_CONTACTS_H__

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "TriMesh.h"
#include "Physics/RigidBody/common/SubData.h"

// sepAxisは-->triangleの方向
// Convex座標系
bool triangleConvexContact(Vector3 &axis,Vector3 &pointsOnTriangle,Vector3 &pointsOnConvex,
							const Vector3 &normal,const Vector3 &p0,const Vector3 &p1,const Vector3 &p2,const float thickness,uint32_t edgeChk,
							const ConvexMesh *meshB);

int trianglesConvexContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, float *distance,
						const TriMesh *meshA, const Transform3 & transformA,
						const ConvexMesh *meshB, const Transform3 & transformB,float distanceThreshold = FLT_MAX );

#endif

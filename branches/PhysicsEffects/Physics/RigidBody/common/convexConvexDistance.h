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

#ifndef __CONVEXCONVEX_DISTANCE_H___
#define __CONVEXCONVEX_DISTANCE_H___

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "TriMesh.h"

float closestConvexConvex( Vector3& normal, Point3 &pointA, Point3 &pointB,
						const ConvexMesh *meshA, const Transform3 & transformA,
						const ConvexMesh *meshB, const Transform3 & transformB,
						float distanceThreshold = FLT_MAX );

#endif // __CONVEXCONVEX_DISTANCE_H___

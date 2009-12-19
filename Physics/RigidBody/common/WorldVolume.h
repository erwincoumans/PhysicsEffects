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

#ifndef __WORLD_VOLUME_H__
#define __WORLD_VOLUME_H__

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/Base/VecInt3.h"

class WorldVolume {
public:
	Vector3 origin;
	Vector3 extent;
	
	inline void	setWorldSize(Vector3 origin_,Vector3 extent_);
	inline Vector3 localToWorldPosition(const VecInt3 &localPosition);
	inline VecInt3 worldToLocalPosition(const Vector3 &worldPosition);
} __attribute__ ((aligned(16)));

inline
void WorldVolume::setWorldSize(Vector3 origin_,Vector3 extent_)
{
	origin = origin_;
	extent = extent_;
}

inline
Vector3 WorldVolume::localToWorldPosition(const VecInt3 &localPosition)
{
	Vector3 sz(65535.0f);
	Vector3 q = divPerElem((Vector3)localPosition,sz);
	return mulPerElem(q,2.0f*extent) + origin - extent;
}

inline
VecInt3 WorldVolume::worldToLocalPosition(const Vector3 &worldPosition)
{
	Vector3 sz(65535.0f);
	Vector3 q = divPerElem(worldPosition - origin + extent,2.0f*extent);
	q = minPerElem(maxPerElem(q,Vector3(0.0f)),Vector3(1.0f)); // clamp 0.0 - 1.0
	q = mulPerElem(q,sz);
	return VecInt3(q);
}

#endif /* __WORLD_VOLUME_H__ */

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

#ifndef __SPU_UTIL_H__
#define __SPU_UTIL_H__

#include <vectormath_aos.h>
using namespace Vectormath::Aos;



static inline Vector3 read_Vector3(const float* p)
{
	Vector3 v;
	loadXYZ(v, p);
	return v;
}

static inline Quat read_Quat(const float* p)
{
	Quat vq;
	loadXYZW(vq, p);
	return vq;
}

static inline void store_Vector3(const Vector3 &src, float* p)
{
	Vector3 v = src;
	storeXYZ(v, p);
}

static inline void store_Quat(const Quat &src, float* p)
{
	Quat vq = src;
	storeXYZW(vq, p);
}

#endif /* __SPU_UTIL_H__ */

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

#ifndef __SPRING_H__
#define __SPRING_H__

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

struct Spring
{
	float length;	// バネの自然長
	float ks;		// バネ係数
	float kd;		// ダンパ係数
	
	uint8_t active;
	
	uint16_t stateIndexA;
	uint16_t stateIndexB;
	
	Vector3 anchorA;
	Vector3 anchorB;
} __attribute__ ((aligned(16)));

#endif

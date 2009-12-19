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

#ifndef __RAY_H__
#define __RAY_H__

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/RigidBody/common/SubData.h"

enum FacetType {
	FacetTypeFront = 0,
	FacetTypeBack  = 1,
	FacetTypeBoth  = 2,
};

struct Ray
{
	// input
	Vector3   startPos;
	Vector3   endPos;
	uint32_t  contactFilterSelf;
	uint32_t  contactFilterTarget;
	uint8_t	  rayGroup;
	uint8_t	  facetType : 2; // LargeTriMesh Only

	// output
	bool      contactFlag : 1;
	uint16_t  contactInstance;
	float     t;
	Vector3   contactPoint;
	Vector3   contactNormal;
	Vector3   rayDir; // endPos - startPos

	// -------------------------------
	// 補助情報

	uint8_t primIdx;
	SubData subData;

	// -------------------------------
	
	Ray()
	{
		reset();
	}

	void reset()
	{
		primIdx = 0;
		subData.type = 0;
		contactFlag = false;
		contactFilterSelf = contactFilterTarget = 0xffffffff;
		rayGroup = 0;
		facetType = FacetTypeFront;
		t = 1.0f;
	}
} __attribute__ ((aligned(16)));

#endif /* __RAY_H__ */

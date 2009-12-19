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

#ifndef __RAYCASTIO_H__
#define __RAYCASTIO_H__

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

///////////////////////////////////////////////////////////////////////////////
// IO Parameter

#ifndef WIN32

// RayCast Event
enum {
	RAYCAST_ASSIGNSTATES = 0,
	RAYCAST_SORTAABB,
	RAYCAST_RAYCAST,
};

struct IOParamBroadPhase {
	uint32_t statesAddr;
	uint32_t numStates;
	uint32_t batchStartState;
	uint32_t numBatchStates;
	uint32_t aabbAddr[3];
	uint32_t numAabb;
	uint32_t worldVolumeAddr;
} __attribute__ ((aligned(16)));

struct IOParamSortAabbs {
	uint32_t numSpu;
	uint32_t buffAddr;
	uint32_t aabbAddr[3];
	uint32_t numAabb;
} __attribute__ ((aligned(16)));

struct IOParamRayCast {
	uint32_t startRay;
	uint32_t numRay;
	uint32_t numStride;
	uint32_t raysAddr;
	uint32_t statesAddr;
	uint32_t collsAddr;
	uint32_t numStates;
	uint32_t aabbAddr[3];
	uint32_t numAabb;
	uint32_t nonContactFlagAddr;
	uint32_t numNonContactFlag;
	uint32_t worldVolumeAddr;
	uint32_t maxRayGroups;
} __attribute__ ((aligned(16)));

#endif

#endif /* __RAYCASTPOINTERS_H__ */


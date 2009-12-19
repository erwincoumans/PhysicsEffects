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

#ifndef __PFX_CONTACT_CACHE_H__
#define __PFX_CONTACT_CACHE_H__

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/PhysicsCommon.h"
#include "SubData.h"

#define PFX_CONTACT_CACHE_SIZE 4
#define PFX_CONTACT_THRESHOLD  0.002f

class PfxContactCache {
private:
	uint32_t numContacts;
	uint32_t info[PFX_CONTACT_CACHE_SIZE];
	float   distance[PFX_CONTACT_CACHE_SIZE];
	Vector3 normal[PFX_CONTACT_CACHE_SIZE];
	Vector3 pointA[PFX_CONTACT_CACHE_SIZE];
	Vector3 pointB[PFX_CONTACT_CACHE_SIZE];
	
	int findNearestContactPoint(const Vector3 &newPoint);
	int sort4ContactPoints(const Vector3 &newPoint,float newDistance);

public:
	PfxContactCache() : numContacts(0)
	{
	}

	void add(float newDistance,Vector3 &newNormal,Vector3 &newPointA,Vector3 &newPointB,uint32_t newInfo);
	
	uint32_t getNumContacts() {return numContacts;}

	uint32_t getInfo(int i) {return info[i];}
	float    getDistance(int i) const {return distance[i];}
	Vector3  getNormal(int i) const {return normal[i];}
	Vector3  getPointA(int i) const {return pointA[i];}
	Vector3  getPointB(int i) const {return pointB[i];}
};

#endif /* __PFX_CONTACT_CACHE_H__ */

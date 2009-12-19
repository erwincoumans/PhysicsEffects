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

#include "PfxContactCache.h"

int PfxContactCache::findNearestContactPoint(const Vector3 &newPoint)
{
	int nearestIdx = -1;
	for(uint32_t i=0;i<numContacts;i++) {
		Vector3 dist = pointA[i] - newPoint;
		float diff = lengthSqr(dist);
		if(diff < PFX_CONTACT_THRESHOLD) {
			nearestIdx = i;
		}
	}
	return nearestIdx;
}

int PfxContactCache::sort4ContactPoints(const Vector3 &newPoint,float newDistance)
{
	int maxPenetrationIndex = -1;
	float maxPenetration = newDistance;

	// 最も深い衝突点は排除対象からはずす
	for(int i=0;i<PFX_CONTACT_CACHE_SIZE;i++) {
		if(distance[i] < maxPenetration) {
			maxPenetrationIndex = i;
			maxPenetration = distance[i];
		}
	}
	
	float res[4] = {0.0f};
	
	// 各点を除いたときの衝突点が作る面積のうち、最も大きくなるものを選択
	if(maxPenetrationIndex != 0) {
		Vector3 a0 = newPoint-pointA[1];
		Vector3 b0 = pointA[3]-pointA[2];
		res[0] = lengthSqr(cross(a0,b0));
	}
 
	if(maxPenetrationIndex != 1) {
		Vector3 a1 = newPoint-pointA[0];
		Vector3 b1 = pointA[3]-pointA[2];
		res[1] = lengthSqr(cross(a1,b1));
	}

	if(maxPenetrationIndex != 2) {
		Vector3 a2 = newPoint-pointA[0];
		Vector3 b2 = pointA[3]-pointA[1];
		res[2] = lengthSqr(cross(a2,b2));
	}

	if(maxPenetrationIndex != 3) {
		Vector3 a3 = newPoint-pointA[0];
		Vector3 b3 = pointA[2]-pointA[1];
		res[3] = lengthSqr(cross(a3,b3));
	}

	int maxIndex = 0;
	float maxVal = res[0];

	if (res[1] > maxVal) {
		maxIndex = 1;
		maxVal = res[1];
	}

	if (res[2] > maxVal) {
		maxIndex = 2;
		maxVal = res[2];
	}

	if (res[3] > maxVal) {
		maxIndex = 3;
		maxVal = res[3];
	}

	return maxIndex;
}

void PfxContactCache::add(float newDistance,Vector3 &newNormal,Vector3 &newPointA,Vector3 &newPointB,uint32_t newInfo)
{
	int replaceId = findNearestContactPoint(newPointA);
	if(replaceId >= 0 && distance[replaceId] < newDistance) {
		return;
	}
	else {
		if(numContacts < 4) {
			replaceId = numContacts++;
		}
		else {
			replaceId = sort4ContactPoints(newPointA,newDistance);
		}
	}
	distance[replaceId] = newDistance;
	normal[replaceId] = newNormal;
	pointA[replaceId] = newPointA;
	pointB[replaceId] = newPointB;
	info[replaceId] = newInfo;
}

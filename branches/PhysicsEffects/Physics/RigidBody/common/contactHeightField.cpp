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

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "HeightFieldFunction.h"

bool contactHeightField(
					const HeightField *heightfield,
					const Point3 &checkPoint,
					Point3 &fieldPoint,
					Vector3 &fieldNormal,
					float &dist)
{
	Vector3 localPoint = heightfield->worldToLocalPosition((Vector3)checkPoint);
	
	if(localPoint[1] < heightfield->getMinHeight() || localPoint[1] > heightfield->getMaxHeight()) {
		return false;
	}

	float h;
	if(!getHeight((*heightfield),localPoint[0],localPoint[2],h)) {
		return false;
	}
	
	dist = heightfield->getScale()[1] * ( localPoint[1] - h );
	
	if(dist < 0.0f) {
		localPoint[1] = h;
		fieldPoint = (Point3)mulPerElem(heightfield->getScale(),(localPoint - 0.5f*Vector3(heightfield->getFieldWidth(),0.0f,heightfield->getFieldDepth())));
//		getNormal((*heightfield),localPoint[0],localPoint[2],fieldNormal);
		fieldNormal = Vector3(0,1,0);
		return true;
	}

	return false;
}

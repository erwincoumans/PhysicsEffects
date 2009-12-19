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

#include <string.h>

#include "HeightFieldFunction.h"





///////////////////////////////////////////////////////////////////////////////
// Internal Function

static inline float interpolate(float a,float b,float x)
{
	return a * (1.0f - x) + b * x;
}



///////////////////////////////////////////////////////////////////////////////
// Public Function

bool getHeight(const HeightField &heightfield,float x,float z,float &h)
{
	// 境界チェック
	if(x < 0 || x > heightfield.fieldData.fieldWidth || z < 0 || z > heightfield.fieldData.fieldDepth) {
		return false;
	}

	int i = (int)x;
	int j = (int)z;

	float xx = x-i;
	float zz = z-j;
	
	
	float h1 = getFieldData(heightfield.fieldData, i  , j  );
	float h2 = getFieldData(heightfield.fieldData, i+1, j  );
	float h3 = getFieldData(heightfield.fieldData, i  , j+1);
	float h4 = getFieldData(heightfield.fieldData, i+1, j+1);

	// 面を分解
	if(xx < zz) {
		h2 = (h1 +h4) * 0.5f;
		h2 += h2 - h3;
	}
	else {
		h3 = (h1 +h4) * 0.5f;
		h3 += h3 - h2;
	}

	// 線形補間
	h = interpolate(interpolate(h1,h2,xx),interpolate(h3,h4,xx),zz);
	
	return true;
}

bool getNormal(const HeightField &heightfield,float x,float z,Vector3 &nml)
{
	int i = (int)x;
	int j = (int)z;

	float xx = x-i;
	float zz = z-j;	
	
	if(i < 0 || i > heightfield.fieldData.fieldWidth || j < 0 || j > heightfield.fieldData.fieldDepth) {
		return false;
	}
	
	// 面の高さを取得
	float h1 = getFieldData(heightfield.fieldData, i  , j  );
	float h2 = getFieldData(heightfield.fieldData, i+1, j  );
	float h3 = getFieldData(heightfield.fieldData, i  , j+1);
	float h4 = getFieldData(heightfield.fieldData, i+1, j+1);
	
	Vector3 p[4] = {
		Vector3(i  ,h1,j  ),
		Vector3(i+1,h2,j  ),
		Vector3(i  ,h3,j+1),
		Vector3(i+1,h4,j+1),
	};
	
	if(xx+zz < 1.0f) {
		p[0] = mulPerElem(p[0],heightfield.getScale());
		p[2] = mulPerElem(p[2],heightfield.getScale());
		p[1] = mulPerElem(p[1],heightfield.getScale());
		nml = normalize(cross(p[2]-p[0],p[1]-p[0]));
	}
	else {
		p[3] = mulPerElem(p[3],heightfield.getScale());
		p[1] = mulPerElem(p[1],heightfield.getScale());
		p[2] = mulPerElem(p[2],heightfield.getScale());
		nml = normalize(cross(p[1]-p[3],p[2]-p[3]));
	}
	
	return true;
}


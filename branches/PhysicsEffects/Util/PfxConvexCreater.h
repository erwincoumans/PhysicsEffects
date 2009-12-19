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

#ifndef __PFX_CONVEX_CREATER_H__
#define __PFX_CONVEX_CREATER_H__

#include "Physics/Base/PhysicsCommon.h"

#include <vector>
using namespace std;

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

enum CHUseKDop {
	CH_USE_6DOP = 3,
	CH_USE_14DOP = 7,
	CH_USE_26DOP = 13,
};

struct CHPlane {
	Vector3 N,S,T;
	float d;

	CHPlane() {}

	CHPlane(const Vector3 &n,const Vector3 &q) {
		N = n;
		d = dot(n,q);
		getPlaneSpace(N,S,T);
	}
	
	float onPlane(const Vector3 &p) const {
		return dot(p,N)-d;
	}
	
	void getPlaneSpace(const Vector3& n, Vector3& p, Vector3& q);
};

struct CHPolygon {
	CHPlane plane;
	vector<Vector3> verts;
};

struct CHMesh {
	vector<float> verts;
	vector<uint16_t> indices;
	
	uint16_t getVertId(Vector3 &v);
	void addTriangle(uint16_t vi0,uint16_t vi1,uint16_t vi2);
};

// 頂点配列から凸包を構成するポリゴン配列を作成
void createConvexPolygons(CHUseKDop useKDop,float *vtx,int numVerts,vector<CHPolygon> &polys);

// ポリゴン配列をメッシュに変換
void convertPolygonToMesh(vector<CHPolygon> &polys,CHMesh &mesh);

#endif /* __PFX_CONVEX_CREATER_H__ */

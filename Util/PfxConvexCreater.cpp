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

#include "Util/PfxConvexCreater.h"
#include <assert.h>
#include <float.h>

const static float CHEpsilon = 1e-04f;
const static uint32_t nKDop = 13;
const static Vector3 vKDopTbl[13] = {
	Vector3(  1.0f,  0.0f,  0.0f),
	Vector3(  0.0f,  1.0f,  0.0f),
	Vector3(  0.0f,  0.0f,  1.0f),
	// 6dop

	normalize(Vector3(  1.0f,  1.0f,  1.0f)),
	normalize(Vector3( -1.0f,  1.0f,  1.0f)),
	normalize(Vector3( -1.0f, -1.0f,  1.0f)),
	normalize(Vector3(  1.0f, -1.0f,  1.0f)),
	// 14dop

	normalize(Vector3(  1.0f,  1.0f,  0.0f)),
	normalize(Vector3( -1.0f,  1.0f,  0.0f)),
	normalize(Vector3(  1.0f,  0.0f,  1.0f)),
	normalize(Vector3(  0.0f,  1.0f,  1.0f)),
	normalize(Vector3( -1.0f,  0.0f,  1.0f)),
	normalize(Vector3(  0.0f, -1.0f,  1.0f)),
	// 26dop
};

void CHPlane::getPlaneSpace(const Vector3& n, Vector3& p, Vector3& q)
{
	if (fabsf(n[2]) > 0.707f) {
		// choose p in y-z plane
		float a = n[1]*n[1] + n[2]*n[2];
		float k = 1.0f/sqrtf(a);
		p[0] = 0;
		p[1] = -n[2]*k;
		p[2] = n[1]*k;
		// set q = n x p
		q[0] = a*k;
		q[1] = -n[0]*p[2];
		q[2] = n[0]*p[1];
	}
	else {
		// choose p in x-y plane
		float a = n[0]*n[0] + n[1]*n[1];
		float k = 1.0f/sqrtf(a);
		p[0] = -n[1]*k;
		p[1] = n[0]*k;
		p[2] = 0;
		// set q = n x p
		q[0] = -n[2]*p[1];
		q[1] = n[2]*p[0];
		q[2] = a*k;
	}
}

// 重複する頂点を削除
// ＊あらかじめ頂点配列はソートされている
void removeSameVertex(vector<Vector3> &verts)
{
	if(verts.empty()) return;

	vector<Vector3>::iterator iVerts = verts.begin();
	Vector3 last = *(iVerts++);
	for(;iVerts!=verts.end();) {
		if(length(last-(*iVerts)) < CHEpsilon) {
			last = *iVerts;
			iVerts = verts.erase(iVerts);
		}
		else {
			last = *iVerts;
			iVerts++;
		}
	}
}


// ポリゴンを平面で切断
int slicePolygonByPlane(CHPolygon &poly,CHPlane &plane,vector<Vector3> &gatherVerts)
{
	uint32_t sliceEdgeId[2];
	uint32_t idcnt = 0;
	uint32_t upper = 0;
	uint32_t lower = 0;

	for(uint32_t i=0;i<poly.verts.size();i++) {
		Vector3 e1 = poly.verts[i];
		Vector3 e2 = poly.verts[(i+1)%poly.verts.size()];

		float d1 = plane.onPlane(e1);
		float d2 = plane.onPlane(e2);

		if(d1 * d2 < 0.0f || d1 == 0.0f) {
			sliceEdgeId[idcnt++] = i;
		}

		if(d1 > 0.0f) {
			upper++;
		}
		else {
			lower++;
		}
	}

	// 全頂点が平面より上側ならこのポリゴンを消去
	if(upper == poly.verts.size()) return -1;

	// 全頂点が平面より下側ならなにもしない
	if(lower == poly.verts.size()) return -2;

	// 切断箇所のチェック
	//if(idcnt != 2) return -3;
	assert(idcnt < 3);

	// 交差判定
	Vector3 insertVtx[2];

	for(uint32_t i=0;i<idcnt;i++) {
		Vector3 e1 = poly.verts[sliceEdgeId[i]];
		Vector3 e2 = poly.verts[(sliceEdgeId[i]+1)%poly.verts.size()];

		Vector3 dir = e2-e1;
		float t = (plane.d-dot(plane.N,e1)) / dot(plane.N,dir);

		if( t >= 0.0f && t <= 1.0f) {
			insertVtx[i] = e1 + t * dir;
		}
		else {
			assert(0); // 必ず交差しているはず
		}
	}

	// 頂点配列の整理
	vector<Vector3> newVerts;

	for(uint32_t i=0;i<poly.verts.size();i++) {
		float d = plane.onPlane(poly.verts[i]);

		if(d <= 0.0f) {
			newVerts.push_back(poly.verts[i]);
		}
	
		if(idcnt>0 && sliceEdgeId[0] == i) {
			newVerts.push_back(insertVtx[0]);
			gatherVerts.push_back(insertVtx[0]);
		}
		else if(idcnt>1 && sliceEdgeId[1] == i) {
			newVerts.push_back(insertVtx[1]);
			gatherVerts.push_back(insertVtx[1]);
		}
	}

	// 重複頂点のチェック
	removeSameVertex(newVerts);

	poly.verts = newVerts;

	// 面を作れないので、ポリゴンを消去
	if(poly.verts.size() < 3) return -1;

	return 0;
}


// 頂点配列を平面上で反時計回りになるように並べ替える
// ＊頂点配列は平面上に存在
void sortGatherVertsByPlane(vector<Vector3> &gatherVerts,CHPlane &plane)
{
	if(gatherVerts.empty()) return;

	vector<Vector3> newVerts;

	// 並べ替え
	{
		float mint = FLT_MAX;
		int minId = 0;
		for(uint32_t i=0;i<gatherVerts.size();i++) {
			float t = dot(plane.S,gatherVerts[i]);
			if(t < mint) {
				mint = t;
				minId = i;
			}
		}

		Vector3 minVtx = gatherVerts[minId];
		newVerts.push_back(minVtx);
		gatherVerts.erase(gatherVerts.begin()+minId);

		while(!gatherVerts.empty()) {
			float minAng = FLT_MAX;
			for(uint32_t i=0;i<gatherVerts.size();i++) {
				Vector3 v = gatherVerts[i]-minVtx;
				float lenSqr = lengthSqr(v);
				if(lenSqr < CHEpsilon * CHEpsilon) {
					minId = i;
					break;
				}
				float ang = dot(v/sqrtf(lenSqr),plane.T);
				if(ang < minAng) {
					minAng = ang;
					minId = i;
				}
			}
			newVerts.push_back(gatherVerts[minId]);
			gatherVerts.erase(gatherVerts.begin()+minId);
		}
	}
	
	// 重複頂点のチェック
	removeSameVertex(newVerts);

	gatherVerts = newVerts;
}

// ポリゴンの面積を計算する
float calcPolygonArea(CHPolygon &poly)
{
	if(poly.verts.size() < 3)
		return 0.0f;

	float area = 0.0f;
	Vector3 p0 = poly.verts[0];
	for(uint32_t i=1;i<poly.verts.size()-1;i++) {
		Vector3 p1 = poly.verts[i];
		Vector3 p2 = poly.verts[i+1];
		Vector3 v = cross(p2-p0,p1-p0);

		float lenSqr = lengthSqr(v);
		if(lenSqr > CHEpsilon * CHEpsilon) {
			area += sqrtf(lenSqr);
		}
	}

	return area;
}

void createConvexPolygons(CHUseKDop useKDop,float *vtx,int numVerts,vector<CHPolygon> &polys)
{
	float kdMin[13],kdMax[13];

	// 26dopの作成
	for(uint32_t i = 0; i < nKDop; i++) {
		float vmin=FLT_MAX,vmax=-FLT_MAX;
		for(uint32_t j = 0; j < (uint32_t)numVerts; j++) {
			Vector3 v(vtx[j*3],vtx[j*3+1],vtx[j*3+2]);
			float proj = dot(v,vKDopTbl[i]);
			if(proj < vmin) vmin = proj;
			if(proj > vmax) vmax = proj;
		}
		kdMin[i] = vmin;
		kdMax[i] = vmax;
	}

	// 初期状態(6dop)の作成
	polys.clear();

	Vector3 boxMin(kdMin[0],kdMin[1],kdMin[2]);
	Vector3 boxMax(kdMax[0],kdMax[1],kdMax[2]);
	
	{ // -X
		CHPolygon poly;
		poly.plane = CHPlane(-vKDopTbl[0],kdMin[0]*vKDopTbl[0]);
		poly.verts.push_back(Vector3(kdMin[0],kdMin[1],kdMin[2]));
		poly.verts.push_back(Vector3(kdMin[0],kdMin[1],kdMax[2]));
		poly.verts.push_back(Vector3(kdMin[0],kdMax[1],kdMax[2]));
		poly.verts.push_back(Vector3(kdMin[0],kdMax[1],kdMin[2]));
		polys.push_back(poly);
	}
	{ // +X
		CHPolygon poly;
		poly.plane = CHPlane(vKDopTbl[0],kdMax[0]*vKDopTbl[0]);
		poly.verts.push_back(Vector3(kdMax[0],kdMin[1],kdMin[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMax[1],kdMin[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMax[1],kdMax[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMin[1],kdMax[2]));
		polys.push_back(poly);
	}
	{ // -Y
		CHPolygon poly;
		poly.plane = CHPlane(-vKDopTbl[1],kdMin[1]*vKDopTbl[1]);
		poly.verts.push_back(Vector3(kdMin[0],kdMin[1],kdMin[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMin[1],kdMin[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMin[1],kdMax[2]));
		poly.verts.push_back(Vector3(kdMin[0],kdMin[1],kdMax[2]));
		polys.push_back(poly);
	}
	{ // +Y
		CHPolygon poly;
		poly.plane = CHPlane(vKDopTbl[1],kdMax[1]*vKDopTbl[1]);
		poly.verts.push_back(Vector3(kdMin[0],kdMax[1],kdMin[2]));
		poly.verts.push_back(Vector3(kdMin[0],kdMax[1],kdMax[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMax[1],kdMax[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMax[1],kdMin[2]));
		polys.push_back(poly);
	}
	{ // -Z
		CHPolygon poly;
		poly.plane = CHPlane(-vKDopTbl[2],kdMin[2]*vKDopTbl[2]);
		poly.verts.push_back(Vector3(kdMin[0],kdMin[1],kdMin[2]));
		poly.verts.push_back(Vector3(kdMin[0],kdMax[1],kdMin[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMax[1],kdMin[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMin[1],kdMin[2]));
		polys.push_back(poly);
	}
	{ // +Z
		CHPolygon poly;
		poly.plane = CHPlane(vKDopTbl[2],kdMax[2]*vKDopTbl[2]);
		poly.verts.push_back(Vector3(kdMin[0],kdMin[1],kdMax[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMin[1],kdMax[2]));
		poly.verts.push_back(Vector3(kdMax[0],kdMax[1],kdMax[2]));
		poly.verts.push_back(Vector3(kdMin[0],kdMax[1],kdMax[2]));
		polys.push_back(poly);
	}

	for(uint32_t k=3;k<(uint32_t)useKDop;k++) {
		CHPlane planes[2] = {
			CHPlane(-vKDopTbl[k],kdMin[k]*vKDopTbl[k]),
			CHPlane( vKDopTbl[k],kdMax[k]*vKDopTbl[k]),
		};

		for(uint32_t p=0;p<2;p++) {
			vector<Vector3> gatherVerts;
			vector<CHPolygon>::iterator iPoly = polys.begin();
			for(;iPoly!=polys.end();) {
				int ret = slicePolygonByPlane(*iPoly,planes[p],gatherVerts);
				if(ret == -1) {
					iPoly = polys.erase(iPoly);
				}
				else {
					iPoly++;
				}
			}
			sortGatherVertsByPlane(gatherVerts,planes[p]);
			if(gatherVerts.size() >= 3) {
				CHPolygon poly;
				poly.plane = planes[p];
				poly.verts.assign(gatherVerts.begin(),gatherVerts.end());
				polys.push_back(poly);
			}
		}
	}

	// 面積０のポリゴンを省く
	{
		vector<CHPolygon>::iterator iPoly = polys.begin();
		for(;iPoly!=polys.end();) {
			float area = calcPolygonArea(*iPoly);
			if(area < CHEpsilon) {
				iPoly = polys.erase(iPoly);
			}
			else {
				iPoly++;
			}
		}
	}
}

uint16_t CHMesh::getVertId(Vector3 &v)
{
	for(uint16_t i=0;i<verts.size()/3;i++) {
		Vector3 vv(verts[i*3],verts[i*3+1],verts[i*3+2]);
		float lenSqr = lengthSqr(v-vv);
		if(lenSqr < CHEpsilon * CHEpsilon) {
			return i;
		}
	}
	
	verts.push_back(v[0]);
	verts.push_back(v[1]);
	verts.push_back(v[2]);
	
	return verts.size()/3-1;
}

void CHMesh::addTriangle(uint16_t vi0,uint16_t vi1,uint16_t vi2)
{
	assert(vi0 != vi1 && vi0 != vi2 && vi1 != vi2);

	indices.push_back(vi0);
	indices.push_back(vi1);
	indices.push_back(vi2);
}

void convertPolygonToMesh(vector<CHPolygon> &polys,CHMesh &mesh)
{
	mesh.verts.clear();
	mesh.indices.clear();
	
	for(uint32_t i=0;i<polys.size();i++) {
		uint16_t vi0 = mesh.getVertId(polys[i].verts[0]);
		for(uint32_t j=1;j<polys[i].verts.size()-1;j++) {
			uint16_t vi1 = mesh.getVertId(polys[i].verts[j]);
			uint16_t vi2 = mesh.getVertId(polys[i].verts[j+1]);
			mesh.addTriangle(vi0,vi1,vi2);
		}
	}
}

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

#ifdef WIN32
	#pragma warning(disable:4530)
#endif

#include <assert.h>
#include <string.h>
#include <float.h>
#include <stack>
#include <list>
#include <vector>
#include <algorithm>

using namespace std;

#include "MeshUtil.h"
#include "UtilCommon.h"
#include "PfxConvexCreater.h"

//#define USE_PERFCOUNTER
#include "Physics/Base/PerfCounter.h"
#include "Physics/Base/SimdFunc.h"

namespace MeshUtil {

const float DEFAULT_FACET_THICKNESS = 0.05f;

///////////////////////////////////////////////////////////////////////////////
// メッシュユーティリティで使用する構造体

struct Vert {
	uint16_t i;
	uint16_t flag;
	float fv[3];

	Vert()
	{
		flag = 0;
	}
};

struct Facet {
	Vert *v[3];
	float area;
	Vector3 aabbMin;
	Vector3 aabbMax;
};

struct TriList {
	Facet *facet;
	TriList *next;

	TriList()
	{
		facet = NULL;
		next = NULL;
	}
};

struct EdgeEntry {
	uint8_t vertIdx[2];
	uint8_t facetIdx[2];
	uint8_t numFacets;
	uint8_t edgeNum[2];
	uint8_t edgeIdx;
	Vector3 dir;
	EdgeEntry *next;
};

///////////////////////////////////////////////////////////////////////////////
// メッシュユーティリティ関数

// 同一頂点を共有する面を探索して整理　※入力情報を書き換えます
void shrinkVertices(float *verts,int &numVerts,unsigned short *indices,int &numIndices)
{
	if(numVerts==0||numIndices==0) return;
	
	// 面積０の面を削除
	for(int i=0;i<numIndices/3;i++) {
		Vector3 pnts[3] = {
			Vector3(verts[indices[i*3  ]*3  ],verts[indices[i*3  ]*3+1],verts[indices[i*3  ]*3+2]),
			Vector3(verts[indices[i*3+1]*3  ],verts[indices[i*3+1]*3+1],verts[indices[i*3+1]*3+2]),
			Vector3(verts[indices[i*3+2]*3  ],verts[indices[i*3+2]*3+1],verts[indices[i*3+2]*3+2]),
		};

		float area = lengthSqr(cross(pnts[1]-pnts[0],pnts[2]-pnts[0]));

		if(area < 0.00001f) {
			for(int j=i+1;j<numIndices/3;j++) {
				indices[(j-1)*3  ] = indices[j*3  ];
				indices[(j-1)*3+1] = indices[j*3+1];
				indices[(j-1)*3+2] = indices[j*3+2];
			}
			numIndices -= 3;
			i--;
		}
	}
	
	// 同一頂点をまとめる
	for(int i=0;i<numVerts-1;i++) {
		for(int j=i+1;j<numVerts;j++) {
			
			// 同一頂点であれば消去
			float dx = verts[i*3  ]-verts[j*3  ];
			float dy = verts[i*3+1]-verts[j*3+1];
			float dz = verts[i*3+2]-verts[j*3+2];
			float lenSqr = dx*dx+dy*dy+dz*dz;

			if(lenSqr < 0.00001f) {
				// インデックス付け替え
				for(int k=0;k<numIndices;k++) {
					if(indices[k] == j) {
						indices[k] = i;
					}
					else if(indices[k] > j) {
						indices[k]--;
					}
				}

				// 頂点配列を詰める
				for(int k=j;k<numVerts-1;k++) {
					verts[k*3  ] = verts[(k+1)*3  ];
					verts[k*3+1] = verts[(k+1)*3+1];
					verts[k*3+2] = verts[(k+1)*3+2];
				}
				numVerts--;
				j--;
			}
		}
	}
}

// 面に含まれる頂点のうち、与えられたエッジに含まれないものを返す
inline
uint8_t findUnusedVertexInEdge(const TriMesh &mesh,uint8_t facetIdx,uint8_t facetEdgeIdx)
{
	assert(facetIdx < mesh.numFacets);
	assert(facetEdgeIdx < 3);
	
	const MeshFacet &facet = mesh.facets[facetIdx];
	const MeshEdge &edge = mesh.edges[facet.edgeIndices[facetEdgeIdx]];
	
	uint8_t v1 = edge.vertIndex[0];
	uint8_t v2 = edge.vertIndex[1];
	
	for(uint8_t i=0;i<3;i++) {
		if(facet.vertIndices[i] != v1 && facet.vertIndices[i] != v2) {
			return facet.vertIndices[i];
		}
	}

	assert(0);
	return 0; // error
}

void convertToTriMesh(TriMesh &mesh,float *verts,int numVerts,unsigned short *indices,int numIndices)
{
	// TriMeshへの変換
	mesh.numVerts = numVerts;
	mesh.numFacets = numIndices / 3;
	mesh.numEdges = 0;

	if(numVerts==0||numIndices==0) return;

	for(uint32_t i=0;i<mesh.numVerts;i++) {
		mesh.verts[i] = Vector3(verts[i*3],verts[i*3+1],verts[i*3+2]);
	}
	
	vector<EdgeEntry*> edgeHead;
	vector<EdgeEntry> edgeList;

	edgeHead.assign(mesh.numFacets*3,NULL);
	edgeList.assign(mesh.numFacets*3,EdgeEntry());

	int ecnt = 0;
	for(uint32_t i=0;i<mesh.numFacets;i++) {
		MeshFacet facet;
		facet.half[0] = facet.half[1] = facet.half[2] = 0.0f;
		facet.center[0] = facet.center[1] = facet.center[2] = 0.0f;
		facet.dirGroup = 0;

		// Indices
		uint8_t vertIds[3] = {
			indices[i*3  ],
			indices[i*3+1],
			indices[i*3+2]
		};
		
		facet.vertIndices[0] = vertIds[0];
		facet.vertIndices[1] = vertIds[1];
		facet.vertIndices[2] = vertIds[2];

		// Normal
		Vector3 p0 = mesh.verts[vertIds[0]];
		Vector3 p1 = mesh.verts[vertIds[1]];
		Vector3 p2 = mesh.verts[vertIds[2]];
		store_Vector3(normalize(cross(p2-p1,p0-p1)),facet.normal);
		facet.thickness = DEFAULT_FACET_THICKNESS;
		
		// Edge
		for(uint32_t v=0;v<3;v++) {
			uint8_t viMin = PFX_MIN(vertIds[v],vertIds[(v+1)%3]);
			uint8_t viMax = PFX_MAX(vertIds[v],vertIds[(v+1)%3]);
			int key = ((0x8da6b343*viMin+0xd8163841*viMax)%(mesh.numFacets*3));
			for(EdgeEntry *e = edgeHead[key];;e=e->next) {
				if(!e) {
					edgeList[ecnt].vertIdx[0] = viMin;
					edgeList[ecnt].vertIdx[1] = viMax;
					edgeList[ecnt].facetIdx[0] = i;
					edgeList[ecnt].numFacets = 1;
					edgeList[ecnt].edgeNum[0] = v;
					edgeList[ecnt].edgeIdx = ecnt;
					edgeList[ecnt].dir = normalize(mesh.verts[viMax]-mesh.verts[viMin]);
					edgeList[ecnt].next = edgeHead[key];
					edgeHead[key] = &edgeList[ecnt];

					MeshEdge edge;
					edge.angle = EDGE_FLAT;
					edge.dirGroup = 0;
					edge.vertIndex[0] = viMin;
					edge.vertIndex[1] = viMax;
					//edge.facetIndex[0] = i;
					//edge.facetIndex[1] = 0xff;

					facet.edgeIndices[v] = ecnt;
					mesh.edges[ecnt] = edge;
					assert(ecnt <= NUMMESHEDGES);
					ecnt++;
					break;
				}

				if(e->vertIdx[0] == viMin && e->vertIdx[1] == viMax) {
                    e->facetIdx[1] = i;
					e->edgeNum[1] = v;
					e->numFacets = 2;
					facet.edgeIndices[v] = e->edgeIdx;
					//mesh.edges[e->edgeIdx].facetIndex[1] = i;
					break;
				}
			}
		}
		mesh.numEdges = ecnt;
		mesh.facets[i] = facet;
	}

	// エッジの角度を算出
	for(uint32_t i=0;i<mesh.numEdges;i++) {
		EdgeEntry &e = edgeList[i];
		MeshEdge &edge = mesh.edges[i];

		if(e.numFacets < 2) {
			edge.angle = EDGE_CONVEX;
			continue;
		}
		
		// エッジを共有する面
		MeshFacet &facet1 = mesh.facets[e.facetIdx[0]];
		MeshFacet &facet2 = mesh.facets[e.facetIdx[1]];

		// 面に含まれるが、このエッジに含まれない点
		uint8_t vIdx1 = findUnusedVertexInEdge(mesh,e.facetIdx[0],e.edgeNum[0]);
		uint8_t vIdx2 = findUnusedVertexInEdge(mesh,e.facetIdx[1],e.edgeNum[1]);

		// エッジの凹凸判定
		Vector3 midPnt = (mesh.verts[vIdx1] + mesh.verts[vIdx2] ) * 0.5f;
		Vector3 pntOnEdge =  mesh.verts[edge.vertIndex[0]];
		
		float chk1 = dot(read_Vector3(facet1.normal),midPnt-pntOnEdge);
		float chk2 = dot(read_Vector3(facet2.normal),midPnt-pntOnEdge);

		if(chk1 < -0.00001f && chk2 < -0.00001f) {
			// 凸エッジ - OK
			edge.angle = EDGE_CONVEX;
		}
		else if(chk1 > 0.00001f && chk2 > 0.00001f) {
			// 凹エッジ - NG
			edge.angle = EDGE_CONCAVE;
		}
		else {
			// 平エッジ - OK
			edge.angle = EDGE_FLAT;
		}
	}

	// 同一方向のエッジを同じdirGroupにまとめる
	{
		uint8_t dirGroup = 1;
		for(uint32_t i=0;i<mesh.numEdges;i++) {
			EdgeEntry &eI = edgeList[i];
			MeshEdge &edgeI = mesh.edges[i];

			if(edgeI.dirGroup > 0) continue;

			edgeI.dirGroup = dirGroup++;

			for(uint32_t j=i+1;j<mesh.numEdges;j++) {
				EdgeEntry &eJ = edgeList[j];
				MeshEdge &edgeJ = mesh.edges[j];

				if(fabsf(dot(eI.dir,eJ.dir)) > 0.99f) {
					edgeJ.dirGroup = edgeI.dirGroup;
				}
			}
		}
	}

	//for(uint32_t i=0;i<mesh.numEdges;i++) {
	//	MESH_UTIL_PRINTF("edge %d angle %d dirGroup %d\n",i,mesh.edges[i].angle,mesh.edges[i].dirGroup);
	//}

	// 同一方向の面を同じdirGroupにまとめる
	{
		uint8_t dirGroup = 1;
		for(uint32_t i=0;i<mesh.numFacets;i++) {
			MeshFacet &facetI = mesh.facets[i];

			if(facetI.dirGroup > 0) continue;

			facetI.dirGroup = dirGroup++;

			for(uint32_t j=i+1;j<mesh.numFacets;j++) {
				MeshFacet &facetJ = mesh.facets[j];

				if(dot(read_Vector3(facetI.normal),read_Vector3(facetJ.normal)) > 0.99f) {
					facetJ.dirGroup = facetI.dirGroup;
				}
			}
		}
	}

	//for(uint32_t i=0;i<mesh.numFacets;i++) {
	//	MESH_UTIL_PRINTF("facet %d dirGroup %d\n",i,mesh.facets[i].dirGroup);
	//}

	mesh.updateAABB();

	//MESH_UTIL_PRINTF("TriMesh %d bytes numVerts %d numEdges %d numFacets %d\n",
	//	sizeof(TriMesh),
	//	mesh.numVerts,mesh.numEdges,mesh.numFacets);
	//for(uint32_t f=0;f<mesh.numFacets;f++) {
	//	MESH_UTIL_PRINTF("facet %2d vertIds %2d %2d %2d edgeIds %2d(%2d,%2d) %2d(%2d,%2d) %2d(%2d,%2d)\n",
	//		f,
	//		mesh.facets[f].vertIndices[0],mesh.facets[f].vertIndices[1],mesh.facets[f].vertIndices[2],
	//		mesh.facets[f].edgeIndices[0],mesh.edges[mesh.facets[f].edgeIndices[0]].vertIndex[0],mesh.edges[mesh.facets[f].edgeIndices[0]].vertIndex[1],
	//		mesh.facets[f].edgeIndices[1],mesh.edges[mesh.facets[f].edgeIndices[1]].vertIndex[0],mesh.edges[mesh.facets[f].edgeIndices[1]].vertIndex[1],
	//		mesh.facets[f].edgeIndices[2],mesh.edges[mesh.facets[f].edgeIndices[2]].vertIndex[0],mesh.edges[mesh.facets[f].edgeIndices[2]].vertIndex[1] );
	//}
}

void createTriangleMesh(TriMesh &mesh,float *verts,int numVerts,unsigned short *indices,int numIndices)
{
	// まずは同一頂点を全てまとめる
	vector<float> newVerts;
	newVerts.assign(numVerts*3,0.0f);
	memcpy(&newVerts[0],verts,sizeof(float)*numVerts*3);

	vector<unsigned short> newIndices;
	newIndices.assign(numIndices,0);
	memcpy(&newIndices[0],indices,sizeof(unsigned short)*numIndices);

	int newNumVerts = numVerts;
	int newNumIndices = numIndices;

	shrinkVertices((float*)&newVerts[0],newNumVerts,&newIndices[0],newNumIndices);

	newVerts.erase(newVerts.begin() + newNumVerts*3,newVerts.end());
	newIndices.erase(newIndices.begin() + newNumIndices,newIndices.end());

	assert(newNumVerts <= NUMMESHVERTICES);
	assert(newNumIndices <= NUMMESHFACETS*3);

	if(!newVerts.empty() && !newIndices.empty()) {
		convertToTriMesh(mesh,(float*)&newVerts[0],newNumVerts,(unsigned short*)&newIndices[0],newNumIndices);
	}
}

void createConvexMesh(ConvexMesh &mesh,float *verts,int numVerts,unsigned short *indices,int numIndices)
{
	// まずは同一頂点を全てまとめる
	vector<float> newVerts;
	newVerts.assign(numVerts*3,0.0f);
	memcpy(&newVerts[0],verts,sizeof(float)*numVerts*3);

	vector<unsigned short> newIndices;
	newIndices.assign(numIndices,0);
	memcpy(&newIndices[0],indices,sizeof(unsigned short)*numIndices);

	int newNumVerts = numVerts;
	int newNumIndices = numIndices;

	shrinkVertices((float*)&newVerts[0],newNumVerts,&newIndices[0],newNumIndices);

	newVerts.erase(newVerts.begin() + newNumVerts*3,newVerts.end());
	newIndices.erase(newIndices.begin() + newNumIndices,newIndices.end());

	assert(newNumVerts <= NUMMESHVERTICES);
	assert(newNumIndices <= NUMMESHFACETS*3);

	mesh.numVerts = newNumVerts;
	for(uint32_t i=0;i<mesh.numVerts;i++) {
		mesh.verts[i] = Vector3(newVerts[i*3],newVerts[i*3+1],newVerts[i*3+2]);
	}

	mesh.numIndices = newNumIndices;
	for(uint32_t i=0;i<mesh.numIndices;i++) {
		mesh.indices[i] = newIndices[i];
	}

	mesh.updateAABB();
}

void createConvexHull(TriMesh &mesh,float *verts,int numVerts)
{
	CHMesh testMesh;
	CHUseKDop dop[3] = {CH_USE_26DOP,CH_USE_14DOP,CH_USE_6DOP};
	int n = 0;
	
	do {
		vector<CHPolygon> polys;
		createConvexPolygons(dop[n++],verts,numVerts,polys);
		convertPolygonToMesh(polys,testMesh);
	} while(testMesh.verts.size() > NUMMESHVERTICES*3 || testMesh.indices.size() > NUMMESHFACETS*3);
	
	assert(testMesh.verts.size() <= NUMMESHVERTICES*3);
	assert(testMesh.indices.size() <= NUMMESHFACETS*3);
	
	convertToTriMesh(mesh,
		(float*)&testMesh.verts[0],testMesh.verts.size()/3,
		(unsigned short*)&testMesh.indices[0],testMesh.indices.size());
}

///////////////////////////////////////////////////////////////////////////////
// HeightField

void createHeightField(HeightField &heightfield,const float *height,int numHeightX,int numHeightZ)
{
	// データの整合性チェック
	assert(numHeightX % BLOCK_SIZE == 0);
	assert(numHeightZ % BLOCK_SIZE == 0);

	// メモリを確保
	heightfield.fieldData.numFieldX = numHeightX;
	heightfield.fieldData.numFieldZ = numHeightZ;
	heightfield.fieldData.fieldWidth = heightfield.fieldData.numFieldX-1;
	heightfield.fieldData.fieldDepth = heightfield.fieldData.numFieldZ-1;
	heightfield.fieldData.numBlockX = heightfield.fieldData.numFieldX / BLOCK_SIZE;
	heightfield.fieldData.numBlockZ = heightfield.fieldData.numFieldZ / BLOCK_SIZE;
	heightfield.fieldData.blocks = (HeightFieldBlock*)PFX_UTIL_ALLOC(16,sizeof(HeightFieldBlock)*heightfield.fieldData.numBlockX * heightfield.fieldData.numBlockZ);

	// 高さデータをブロックに分割
	float maxHeight = -FLT_MAX;
	float minHeight =  FLT_MAX;
	for(int j=0;j<numHeightZ;j++) {
		for(int i=0;i<numHeightX;i++) {
			float h = height[j*numHeightX+i];
			setFieldData(heightfield.fieldData,i,j,h);
			maxHeight = PFX_MAX(maxHeight,h);
			minHeight = PFX_MIN(minHeight,h);
		}
	}
	
	heightfield.fieldData.maxHeight = maxHeight;
	heightfield.fieldData.minHeight = minHeight;
}

void releaseHeightField(HeightField &heightfield)
{
	PFX_UTIL_FREE(heightfield.fieldData.blocks);
}

///////////////////////////////////////////////////////////////////////////////
// ラージメッシュ

// ラージメッシュの生成（空）
void createLargeTriMesh(LargeTriMesh &lmesh,uint16_t maxIslands)
{
	lmesh.numIslands = 0;
	lmesh.maxIslands = maxIslands;

	lmesh.aabbList = (PfxAABB16*)PFX_UTIL_ALLOC(128,sizeof(PfxAABB16)*maxIslands);
	lmesh.islands = (TriMesh*)PFX_UTIL_ALLOC(128,sizeof(TriMesh)*maxIslands);

	PRINTF("create LargeTriMesh size = %dbytes\n",sizeof(float)*3*maxIslands*2+sizeof(TriMesh)*maxIslands+sizeof(LargeTriMesh));
}

// ラージメッシュにアイランドを追加　アイランドのインデックスを返す
int addIslandToLargeTriMesh(LargeTriMesh &lmesh,float *verts,int numVerts,unsigned short *indices,int numIndices)
{
	assert(lmesh.numIslands < lmesh.maxIslands);

	int newIsland = lmesh.numIslands++;

	TriMesh &island = lmesh.islands[newIsland];

	createTriangleMesh(island,verts,numVerts,indices,numIndices);
	
	// アイランドローカルのAABBを計算
	if(island.numFacets > 0) {
		Vector3 aabbMin,aabbMax;
		island.updateIslandAABB(aabbMin,aabbMax);

		VecInt3 aabbMinL = lmesh.getLocalPositionFloor(aabbMin);
		VecInt3 aabbMaxL = lmesh.getLocalPositionCeil(aabbMax);

		setXMin(lmesh.aabbList[newIsland],aabbMinL.getX());
		setXMax(lmesh.aabbList[newIsland],aabbMaxL.getX());
		setYMin(lmesh.aabbList[newIsland],aabbMinL.getY());
		setYMax(lmesh.aabbList[newIsland],aabbMaxL.getY());
		setZMin(lmesh.aabbList[newIsland],aabbMinL.getZ());
		setZMax(lmesh.aabbList[newIsland],aabbMaxL.getZ());
	}
	
	return newIsland;
}

int addIslandToLargeTriMesh(LargeTriMesh &lmesh,TriMesh &island)
{
	assert(lmesh.numIslands < lmesh.maxIslands);
	assert(island.numFacets <= NUMMESHFACETS);

	int newIsland = lmesh.numIslands++;
	lmesh.islands[newIsland] = island;
	
	// アイランドローカルのAABBを計算
	if(island.numFacets > 0) {
		Vector3 aabbMin,aabbMax;
		island.updateIslandAABB(aabbMin,aabbMax);

		VecInt3 aabbMinL = lmesh.getLocalPositionFloor(aabbMin);
		VecInt3 aabbMaxL = lmesh.getLocalPositionCeil(aabbMax);
		
		setXMin(lmesh.aabbList[newIsland],aabbMinL.getX());
		setXMax(lmesh.aabbList[newIsland],aabbMaxL.getX());
		setYMin(lmesh.aabbList[newIsland],aabbMinL.getY());
		setYMax(lmesh.aabbList[newIsland],aabbMaxL.getY());
		setZMin(lmesh.aabbList[newIsland],aabbMinL.getZ());
		setZMax(lmesh.aabbList[newIsland],aabbMaxL.getZ());
	}
	
	return newIsland;
}

// ラージメッシュの破棄
void releaseLargeTriMesh(LargeTriMesh &lmesh)
{
	lmesh.numIslands = lmesh.maxIslands = 0;

	PFX_UTIL_FREE(lmesh.aabbList);
	PFX_UTIL_FREE(lmesh.islands);
}

// アイランドの更新
void updateIslandInLargeTriMesh(LargeTriMesh &lmesh,uint8_t islandIndex,float *verts,int numVerts)
{
	assert(islandIndex < lmesh.numIslands);

	TriMesh &island = lmesh.islands[islandIndex];

	assert(island.numVerts == numVerts);

	if(island.numFacets == 0) return;

	// 頂点座標を更新
	for(int i=0;i<island.numVerts;i++) {
		island.verts[i] = read_Vector3(&verts[i*3]);
	}

	// 法線を更新
	for(int f=0;f<island.numFacets;f++) {
		MeshFacet &facet = island.facets[f];

		Vector3 pnts[3] = {
			island.verts[facet.vertIndices[0]],
			island.verts[facet.vertIndices[1]],
			island.verts[facet.vertIndices[2]],
		};

		store_Vector3(normalize(cross(pnts[2]-pnts[1],pnts[0]-pnts[1])),facet.normal);
	}

	island.updateAABB();

	// アイランドローカルのAABBを計算
	if(island.numFacets > 0) {
		Vector3 aabbMin,aabbMax;
		island.updateIslandAABB(aabbMin,aabbMax);

		VecInt3 aabbMinL = lmesh.getLocalPositionFloor(aabbMin);
		VecInt3 aabbMaxL = lmesh.getLocalPositionCeil(aabbMax);
		
		setXMin(lmesh.aabbList[islandIndex],aabbMinL.getX());
		setXMax(lmesh.aabbList[islandIndex],aabbMaxL.getX());
		setYMin(lmesh.aabbList[islandIndex],aabbMinL.getY());
		setYMax(lmesh.aabbList[islandIndex],aabbMaxL.getY());
		setZMin(lmesh.aabbList[islandIndex],aabbMinL.getZ());
		setZMax(lmesh.aabbList[islandIndex],aabbMaxL.getZ());
	}
}

// ラージメッシュの複製
void copyLargeTriMesh(const LargeTriMesh &srcLmesh,LargeTriMesh &dstLmesh)
{
	memcpy(dstLmesh.aabbList,srcLmesh.aabbList,sizeof(PfxAABB16)*srcLmesh.numIslands);
	memcpy(dstLmesh.islands,srcLmesh.islands,sizeof(TriMesh)*srcLmesh.numIslands);
	dstLmesh.numIslands = srcLmesh.numIslands;
}

// ラージメッシュの更新
void updateLargeTriMesh(LargeTriMesh &lmesh)
{
	if(lmesh.numIslands == 0) return;

	// アイランドのバウンディングボリュームを更新
	for(uint32_t i=0;i<lmesh.numIslands;i++) {
		TriMesh &island = lmesh.islands[i];

		Vector3 aabbMin,aabbMax;
		island.updateIslandAABB(aabbMin,aabbMax);

		VecInt3 aabbMinL = lmesh.getLocalPositionFloor(aabbMin);
		VecInt3 aabbMaxL = lmesh.getLocalPositionCeil(aabbMax);

		setXMin(lmesh.aabbList[i],aabbMinL.getX());
		setXMax(lmesh.aabbList[i],aabbMaxL.getX());
		setYMin(lmesh.aabbList[i],aabbMinL.getY());
		setYMax(lmesh.aabbList[i],aabbMaxL.getY());
		setZMin(lmesh.aabbList[i],aabbMinL.getZ());
		setZMax(lmesh.aabbList[i],aabbMaxL.getZ());
	}

}

///////////////////////////////////////////////////////////////////////////////
// ラージメッシュの作成（自動アイランド分割）

struct IslandsStack {
	vector<Facet*> facets;
	Vector3 center;
	Vector3 half;

	IslandsStack() {}
	IslandsStack(	vector<Facet*> &facets_,
					Vector3 center_,
					Vector3 half_)
	{
		facets = facets_;
		center = center_;
		half = half_;
	}
};

void divideMeshes(const LargeTriMeshConfig &config,
				  vector< vector<Facet*> > &islands,vector<Facet*> &facetList,
				  const Vector3 &center,const Vector3 &half)
{
	stack<IslandsStack> ms;
	
	ms.push(IslandsStack(facetList,center,half));

	float halfLimit = length(half) * config.islandsRatio;

	do {
		// スタックから取り出す
		IslandsStack &sd = ms.top();

		// 含まれる面数が規定値以下であれば、アイランドに登録
		if((sd.facets.size() <= NUMMESHFACETS && length(sd.half) < halfLimit) || 
			(sd.facets.size() <= config.numFacetsLimit ) ) {
			islands.push_back(sd.facets);
			ms.pop();
			continue;
		}

		// 分割後のアイランド
		IslandsStack newIslands[2];

		// 最も適切と思われる分離軸を探す
		int divAxis;
		{
			if(sd.half[0] > sd.half[1]) {
				if(sd.half[0] > sd.half[2]) {
					divAxis = 0;
				}
				else if(sd.half[1] > sd.half[2]) {
					divAxis = 1;
				}
				else {
					divAxis = 2;
				}
			}
			else {
				if(sd.half[1] > sd.half[2]) {
					divAxis = 1;
				}
				else if(sd.half[0] > sd.half[2]) {
					divAxis = 0;
				}
				else {
					divAxis = 2;
				}
			}
		}

		// 中心で分割して、さらに再帰的に処理を続ける
		{
			Vector3 movCenter(0.0f);
			movCenter[divAxis] = 0.5f*sd.half[divAxis];
			
			newIslands[0].center = sd.center + movCenter;
			newIslands[1].center = sd.center - movCenter;
			newIslands[0].half = sd.half;
			newIslands[0].half[divAxis] *= 0.5f;
			newIslands[1].half = newIslands[0].half;
		}
		
		// 新しいAABBに含まれる面をそれぞれの領域に分配
		for(uint32_t f=0;f<sd.facets.size();f++) {
			// 面のAABB
			Vector3 facetCenter = (sd.facets[f]->aabbMin + sd.facets[f]->aabbMax) * 0.5f;
			Vector3 facetHalf = (sd.facets[f]->aabbMax - sd.facets[f]->aabbMin) * 0.5f;

			// AABB判定
			if(!(fabsf(newIslands[0].center[divAxis]-facetCenter[divAxis]) > (newIslands[0].half[divAxis]+facetHalf[divAxis]))) {
				// この面はAABB0に登録
				newIslands[0].facets.push_back(sd.facets[f]);
			}
			else {
				// この面はAABB1に登録
				newIslands[1].facets.push_back(sd.facets[f]);
			}
		}

		ms.pop();

		// 分割した面をチェック
		//PRINTF("newIsland 0\n");
		//for(int i=0;i<newIslands[0].facets.size();i++) {
		//	Facet *f = newIslands[0].facets[i];
		//	PRINTF("  f%d v%d %d %d\n",i,f->v[0]->i,f->v[1]->i,f->v[2]->i);
		//}
		//PRINTF("newIsland 1\n");
		//for(int i=0;i<newIslands[1].facets.size();i++) {
		//	Facet *f = newIslands[1].facets[i];
		//	PRINTF("  f%d v%d %d %d\n",i,f->v[0]->i,f->v[1]->i,f->v[2]->i);
		//}

		if(newIslands[0].facets.size() < newIslands[1].facets.size()) {
			if(newIslands[1].facets.size() > 0) ms.push(newIslands[1]);
			if(newIslands[0].facets.size() > 0) ms.push(newIslands[0]);
		}
		else {
			if(newIslands[0].facets.size() > 0) ms.push(newIslands[0]);
			if(newIslands[1].facets.size() > 0) ms.push(newIslands[1]);
		}
	} while(!ms.empty());
}

void createIsland(TriMesh &island,vector<Facet*> &facets)
{
	uint32_t vertsFlag[(0xff*NUMMESHFACETS*3+31)/32];
	memset(vertsFlag,0,sizeof(uint32_t)*((0xff*NUMMESHFACETS*3+31)/32));
	
	vector<float> verts;
	vector<unsigned short> indices;

	int vcnt = 0;
	for(uint32_t f=0;f<facets.size();f++) {
		for(int iv=0;iv<3;iv++) {
			Vert *vert = facets[f]->v[iv];
			uint32_t idx = vert->i;
			uint32_t mask = 1 << (idx & 31);
			if((vertsFlag[idx>>5] & mask) == 0) {
                vertsFlag[idx>>5] |= mask;
				verts.push_back(vert->fv[0]);
				verts.push_back(vert->fv[1]);
				verts.push_back(vert->fv[2]);
				vert->flag = vcnt;// 新しいインデックス
				vcnt++;
			}

			indices.push_back(vert->flag);
		}
	}

	convertToTriMesh(island,(float*)&verts[0],verts.size()/3,(unsigned short*)&indices[0],indices.size());
}

void autoGenerateLargeTriMesh(const LargeTriMeshConfig &config,LargeTriMesh &lmesh,float *verts,int numVerts,unsigned short *indices,int numIndices)
{
	if(numVerts == 0 || numIndices == 0) return;

	PerfCounter pc;

	vector<Vert> vertList;		// 頂点配列
	vector<Facet> facetList;	// 面配列
	vector<TriList> triEntry;
	vector<TriList*> triHead;	// 頂点から面への参照リスト

	int numFacets = numIndices/3;

	// 同一頂点を整理
	pc.countBegin("organizeTriangles0");
	vertList.assign(numVerts,Vert());
	for(uint32_t i=0;i<(uint32_t)numVerts;i++) {
		vertList[i].i = i;
		vertList[i].fv[0] = verts[i*3];
		vertList[i].fv[1] = verts[i*3+1];
		vertList[i].fv[2] = verts[i*3+2];
		//PRINTF("org vert %f %f %f\n",verts[i*3],verts[i*3+1],verts[i*3+2]);
	}

	//facetList.assign(numFacets,Facet());
	for(uint32_t i=0;i<(uint32_t)numFacets;i++) {
		Vector3 pnts[3] = {
			Vector3(verts[indices[i*3  ]*3  ],verts[indices[i*3  ]*3+1],verts[indices[i*3  ]*3+2]),
			Vector3(verts[indices[i*3+1]*3  ],verts[indices[i*3+1]*3+1],verts[indices[i*3+1]*3+2]),
			Vector3(verts[indices[i*3+2]*3  ],verts[indices[i*3+2]*3+1],verts[indices[i*3+2]*3+2]),
		};

		// 面積が０の面を排除
		float area = lengthSqr(cross(pnts[1]-pnts[0],pnts[2]-pnts[0]));

		if(area < 0.00001f) continue;

		Facet facet;
		facet.v[0] = &vertList[indices[i*3  ]];
		facet.v[1] = &vertList[indices[i*3+1]];
		facet.v[2] = &vertList[indices[i*3+2]];
		facet.area = area;

		facetList.push_back(facet);
		//PRINTF("org facet(%d) %d %d %d\n",i,indices[i*3],indices[i*3+1],indices[i*3+2]);
	}
	numFacets = facetList.size();
	pc.countEnd();

	// 頂点から面への参照リストを作成
	pc.countBegin("organizeTriangles1");
	int cnt = 0;
	triEntry.assign(numIndices,TriList());
	triHead.assign(numIndices,NULL);

	for(uint32_t i=0;i<(uint32_t)numFacets;i++) {
		for(uint32_t v=0;v<3;v++) {
			uint32_t vertId = facetList[i].v[v]->i;
			triEntry[cnt].facet = &facetList[i];
			triEntry[cnt].next = triHead[vertId];
			triHead[vertId] = &triEntry[cnt++];
		}
	}
	pc.countEnd();
	
	pc.countBegin("organizeTriangles2");
	{
		for(uint32_t i=0;i<(uint32_t)numVerts;i++) {
			if(vertList[i].flag == 1) continue;
			for(uint32_t j=i+1;j<(uint32_t)numVerts;j++) {
				if(vertList[j].flag == 1) continue;

				float dx = vertList[i].fv[0]-vertList[j].fv[0];
				float dy = vertList[i].fv[1]-vertList[j].fv[1];
				float dz = vertList[i].fv[2]-vertList[j].fv[2];
				float lenSqr = dx*dx+dy*dy+dz*dz;

				if(lenSqr < 0.00001f) {
					//PRINTF("same position %d,%d\n",i,j);
					vertList[j].flag = 1; // 同一点なのでフラグを立てる
					for(TriList *f=triHead[j];f!=NULL;f=f->next) {
						for(int k=0;k<3;k++) {
							if(f->facet->v[k] == &vertList[j]) {
								f->facet->v[k] = &vertList[i]; // 頂点を付け替える
								break;
							}
						}
					}
				}
			}
		}
	}
	pc.countEnd();

	//for(unsigned int i=0;i<facetList.size();i++) {
	//	PRINTF("facet(%d) %d %d %d\n",i,facetList[i].v[0]->i,facetList[i].v[1]->i,facetList[i].v[2]->i);
	//}

	// 面の面積によって３種類に分類する
	pc.countBegin("split 3 groups1");
	float areaMin=FLT_MAX,areaMax=-FLT_MAX;
	for(uint32_t f=0;f<(uint32_t)numFacets;f++) {
		Vector3 pnts[3] = {
			Vector3(facetList[f].v[0]->fv[0],facetList[f].v[0]->fv[1],facetList[f].v[0]->fv[2]),
			Vector3(facetList[f].v[1]->fv[0],facetList[f].v[1]->fv[1],facetList[f].v[1]->fv[2]),
			Vector3(facetList[f].v[2]->fv[0],facetList[f].v[2]->fv[1],facetList[f].v[2]->fv[2]),
		};
		//facetList[f].area = lengthSqr(cross(pnts[1]-pnts[0],pnts[2]-pnts[0]));
		//PRINTF("facet %d area %f\n",f,facetList[f].area);
		areaMin = PFX_MIN(areaMin,facetList[f].area);
		areaMax = PFX_MAX(areaMax,facetList[f].area);

		// 面のAABBを算出
		facetList[f].aabbMin = minPerElem(pnts[2],minPerElem(pnts[1],pnts[0]));
		facetList[f].aabbMax = maxPerElem(pnts[2],maxPerElem(pnts[1],pnts[0]));
	}
	pc.countEnd();

	pc.countBegin("split 3 groups2");
	float areaDiff = (areaMax-areaMin)/3.0f;
	float areaLevel0,areaLevel1;
	areaLevel0 = areaMin + areaDiff;
	areaLevel1 = areaMin + areaDiff * 2.0f;

	vector<Facet*> facetsLv[3];

	for(uint32_t f=0;f<(uint32_t)numFacets;f++) {
		float area = facetList[f].area;

		int lv;
		if(area < areaLevel0) {
			lv = 0;
		}
		else if(area > areaLevel1) {
			lv = 2;
		}
		else {
			lv = 1;
		}

		facetsLv[lv].push_back(&facetList[f]);
	}
	pc.countEnd();

	// アイランドの配列
	vector< vector<Facet*> > islands;

	// それぞれのレベル毎にTriMeshを作成
	pc.countBegin("divideMeshes");
	for(uint32_t lv=0;lv<3;lv++) {
		if(facetsLv[lv].empty()) continue;

		// 全体のAABBを求める
		Vector3 aabbMin,aabbMax,center,half;
		aabbMin = facetsLv[lv][0]->aabbMin;
		aabbMax = facetsLv[lv][0]->aabbMax;
		for(uint32_t f=1;f<facetsLv[lv].size();f++) {
			aabbMin = minPerElem(facetsLv[lv][f]->aabbMin,aabbMin);
			aabbMax = maxPerElem(facetsLv[lv][f]->aabbMax,aabbMax);
		}
		center = ( aabbMin + aabbMax ) * 0.5f;
		half = ( aabbMax - aabbMin ) * 0.5f;

		// 再帰的に処理
		divideMeshes(config,islands,facetsLv[lv],center,half);
	}
	
	// Check Islands
	//for(int i=0;i<islands.size();i++) {
	//	PRINTF("island %d\n",i);
	//	for(int f=0;f<islands[i].size();f++) {
	//		Facet *facet = islands[i][f];
	//		PRINTF("   %d %d %d\n",facet->v[0]->i,facet->v[1]->i,facet->v[2]->i);
	//	}
	//}

	pc.countEnd();

	// LargeTriMeshの生成
	pc.countBegin("createMesh");
	if(islands.size() > 0 && islands.size() <= 0xff) {
		createLargeTriMesh(lmesh,islands.size());
		int maxFacets=0,maxVerts=0,maxEdges=0;
		
		for(uint32_t i=0;i<islands.size();i++) {
			TriMesh island;
			createIsland(island,islands[i]);
			addIslandToLargeTriMesh(lmesh,island);
			maxFacets = PFX_MAX(maxFacets,island.numFacets);
			maxVerts = PFX_MAX(maxVerts,island.numVerts);
			maxEdges = PFX_MAX(maxEdges,island.numEdges);
		}
		
		PRINTF("generate completed!\n\tinput mesh verts %d facets %d\n\tislands %d max facets %d verts %d edges %d\n",
			numVerts,numIndices/3,
			lmesh.numIslands,maxFacets,maxVerts,maxEdges);
		PRINTF("\tsizeof(LargeTriMesh) %d sizeof(TriMesh) %d\n",sizeof(LargeTriMesh),sizeof(TriMesh));

		//for(uint32_t i=0;i<islands.size();i++) {
		//	SortData &aabb = lmesh.aabbList[i];
		//	PRINTF("%3d island %3d key %5d X %5d %5d Y %5d %5d Z %5d %5d\n",i,IslandId(aabb),Key(aabb),
		//		XMin(aabb),XMax(aabb),YMin(aabb),YMax(aabb),ZMin(aabb),ZMax(aabb));
		//}
	}
	else {
		PRINTF("islands overflow! %d/%d\n",islands.size(),0xff);
	}
	pc.countEnd();
}

} // namespace MeshUtil

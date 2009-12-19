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

#ifndef __MESH_UTILITY_H__
#define __MESH_UTILITY_H__

#include "Physics/RigidBody/common/TriMesh.h"
#include "Physics/RigidBody/common/HeightField.h"

namespace MeshUtil {

///////////////////////////////////////////////////////////////////////////////
// メッシュユーティリティ関数

// ※制限事項
// ・エッジに３つ以上の面が接続
// ・面積が０の面

// トライアングルメッシュの作成
void createTriangleMesh(TriMesh &mesh,float *verts,int numVerts,unsigned short *indices,int numIndices);

// Convexメッシュの作成
void createConvexMesh(ConvexMesh &mesh,float *verts,int numVerts,unsigned short *indices,int numIndices);

// 頂点配列を囲む凸包を作成
void createConvexHull(ConvexMesh &mesh,float *verts,int numVerts);

///////////////////////////////////////////////////////////////////////////////
// ラージメッシュの作成（手動でアイランドを登録）

// ラージメッシュの生成
void createLargeTriMesh(LargeTriMesh &lmesh,uint16_t maxMeshes);

// ラージメッシュにアイランドを追加、アイランドのインデックスを返す
int addIslandToLargeTriMesh(LargeTriMesh &lmesh,float *verts,int numVerts,unsigned short *indices,int numIndices);
int addIslandToLargeTriMesh(LargeTriMesh &lmesh,TriMesh &island);

// ラージメッシュの破棄
void releaseLargeTriMesh(LargeTriMesh &lmesh);

// ラージメッシュアイランドの更新
// ※更新メッシュの頂点座標数などノーチェックなので注意
void updateIslandInLargeTriMesh(LargeTriMesh &lmesh,uint8_t islandIndex,float *verts,int numVerts);

// ラージメッシュの複製
// ※コピー先はcreateLargeTriMesh()であらかじめメモリを確保してください
void copyLargeTriMesh(const LargeTriMesh &srcLmesh,LargeTriMesh &dstLmesh);

// ラージメッシュの更新
// ※アイランドを追加・更新した後はかならずupdateLargeTriMesh()を実行してください。
void updateLargeTriMesh(LargeTriMesh &lmesh);

///////////////////////////////////////////////////////////////////////////////
// ラージメッシュの作成（自動アイランド分割）
// ※破棄するときはreleaseLargeTriMesh()を使用してください。

// 分割の停止条件
struct LargeTriMeshConfig {
	uint32_t numFacetsLimit;	// アイランド面数閾値
	float islandsRatio; 		// シーン全体に対するアイランドAABBサイズ比率

	LargeTriMeshConfig()
	{
		numFacetsLimit = 15;
		islandsRatio = 0.2f;
	}
};

void autoGenerateLargeTriMesh(const LargeTriMeshConfig &config,LargeTriMesh &lmesh,float *verts,int numVerts,unsigned short *indices,int numIndices);

///////////////////////////////////////////////////////////////////////////////
// ハイトフィールドユーティリティ関数

// 与えられた高さデータからハイトフィールドを生成して返します。
// 高さデータは小ブロックに分割されて格納されます。
// 高さデータの幅と奥行きがブロックで割り切れるようしてください。
// ハイトフィールドを使い終わった後はreleaseHeightField()で破棄してください。

void createHeightField(HeightField &heightfield,const float *height,int numHeightX,int numHeightZ);
void releaseHeightField(HeightField &heightfield);

} // namespace MeshUtil

#endif /* __MESH_UTILITY_H__ */

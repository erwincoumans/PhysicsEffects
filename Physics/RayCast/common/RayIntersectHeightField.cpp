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

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/Base/PhysicsCommon.h"
#include "Physics/RigidBody/common/CollObject.h"
#include "Physics/RigidBody/common/HeightField.h"
#include "Ray.h"

#define HFIELD_RAY_CACHE	4
#define HEPSILON 0.00001f

//#define CACHE_DEBUG_ENABLE
//#define HEIGHTFIELD_X 128
//#define HEIGHTFIELD_Z 128
#define HFIELD_IDX(i,j)		((j)*HEIGHTFIELD_X+(i))
#define HFIELD_VEC3(i,j,k)	(((j)*(HEIGHTFIELD_X)+(i))*3+(k))
#define HFIELD_VEC2(i,j,k)	(((j)*(HEIGHTFIELD_X)+(i))*2+(k))



///////////////////////////////////////////////////////////////////////////////
// Global

HeightField *_heightfield = NULL;

// for DEBUG
#ifdef CACHE_DEBUG_ENABLE

uint32_t fieldFlags[HEIGHTFIELD_X*HEIGHTFIELD_Z] __attribute__ ((aligned(128)));
Vector3 rayCellsStart[200];
Vector3 rayCellsEnd[200];
Vector3 rayContactPoint;
int rayCellsCount;

#endif

///////////////////////////////////////////////////////////////////////////////
// Cache Data

#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE

struct cacheFlag{
	uint8_t flag;// 0:none 1:exist 2:keep
	uint8_t idx;
};

struct HeightFieldCache {
	bool	firstBlock;

	int		curBi,curBj;
	int		curBlockCount;
	int		curBlockBi[HFIELD_RAY_CACHE];
	int		curBlockBj[HFIELD_RAY_CACHE];

	cacheFlag	*cacheTable  __attribute__ ((aligned(16)));
	HeightFieldBlock *cacheData __attribute__ ((aligned(16)));
} hfc;

static void  initializeHeightFieldCache(); 							// ハイトフィールドのキャッシュを初期化
static void  releaseHeightFieldCache();								// ハイトフィールドのキャッシュを開放
static void  initBlockData(int bi,int bj,int stepX,int stepZ);		// キャッシュの初期化
static void  updateBlockData(int bi,int bj,int stepX,int stepZ);	// キャッシュにブロックデータを読み込む
static float getCachedFieldData(int i,int j);						// キャッシュから高さを読み込む

#endif

///////////////////////////////////////////////////////////////////////////////
// Cache Function

#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE

static void  initializeHeightFieldCache()
{
	hfc.firstBlock = true;
	hfc.cacheTable = (cacheFlag*)gPool.allocate(_heightfield->fieldData.numBlockX*_heightfield->fieldData.numBlockZ*sizeof(cacheFlag),HeapManager::ALIGN16);
	hfc.cacheData = (HeightFieldBlock*)gPool.allocate(sizeof(HeightFieldBlock)*HFIELD_RAY_CACHE,HeapManager::ALIGN128);
}

static void  releaseHeightFieldCache()
{
	gPool.deallocate(hfc.cacheData);
	gPool.deallocate(hfc.cacheTable);
}

static void  initBlockData(int bi,int bj,int stepX,int stepZ)
{
	int nextBi = bi+stepX;
	int nextBj = bj+stepZ;

	int nextBlockCount=0;
	int nextBlockBi[HFIELD_RAY_CACHE];
	int nextBlockBj[HFIELD_RAY_CACHE];
	
	{
		nextBlockBi[nextBlockCount] = bi;
		nextBlockBj[nextBlockCount] = bj;
		nextBlockCount++;
	}
	
	if(nextBi >= 0 && nextBi < _heightfield->fieldData.numBlockX) {
		nextBlockBi[nextBlockCount] = nextBi;
		nextBlockBj[nextBlockCount] = bj;
		nextBlockCount++;
	}

	if(nextBj >= 0 && nextBj < _heightfield->fieldData.numBlockZ) {
		nextBlockBi[nextBlockCount] = bi;
		nextBlockBj[nextBlockCount] = nextBj;
		nextBlockCount++;
	}
	
	if((nextBi >= 0 && nextBi < _heightfield->fieldData.numBlockX) && (nextBj >= 0 && nextBj < _heightfield->fieldData.numBlockZ)) {
		nextBlockBi[nextBlockCount] = nextBi;
		nextBlockBj[nextBlockCount] = nextBj;
		nextBlockCount++;
	}
	
	// キャッシュテーブルのクリア
	__builtin_memset(hfc.cacheTable,0,_heightfield->fieldData.numBlockX*_heightfield->fieldData.numBlockZ*sizeof(cacheFlag));

	// バッファに読み込む
	for(int i=0;i<nextBlockCount;i++) {
		// ロード＆ウェイト
		int idx = nextBlockBj[i] * _heightfield->fieldData.numBlockX + nextBlockBi[i];

		memcpy(
			&hfc.cacheData[i],						// キャッシュブロック
			_heightfield->fieldData.blocks[idx],	// フィールドブロック
			sizeof(HeightFieldBlock));				// サイズ

		hfc.cacheTable[idx].idx = i;
		hfc.cacheTable[idx].flag = 1;
	}


	
	hfc.curBi = bi;
	hfc.curBj = bj;
	hfc.curBlockCount = nextBlockCount;

	__builtin_memcpy(hfc.curBlockBi,nextBlockBi,sizeof(int)*nextBlockCount);
	__builtin_memcpy(hfc.curBlockBj,nextBlockBj,sizeof(int)*nextBlockCount);
}

static void  updateBlockData(int bi,int bj,int stepX,int stepZ)
{
	int nextBi = bi+stepX;
	int nextBj = bj+stepZ;

	int	emptyBuff[HFIELD_RAY_CACHE]={-1};
	
	int nextBlockCount=0;
	int nextBlockBi[HFIELD_RAY_CACHE];
	int nextBlockBj[HFIELD_RAY_CACHE];
	
	// 重複バッファのチェック
	{
		int idx = bj * _heightfield->fieldData.numBlockX + bi;
		if(hfc.cacheTable[idx].flag > 0)
			hfc.cacheTable[idx].flag = 2;
		nextBlockBi[nextBlockCount] = bi;
		nextBlockBj[nextBlockCount] = bj;
		nextBlockCount++;
	}
	
	if(nextBi >= 0 && nextBi < _heightfield->fieldData.numBlockX) {
		int idx = bj * _heightfield->fieldData.numBlockX + nextBi;
		if(hfc.cacheTable[idx].flag > 0)
			hfc.cacheTable[idx].flag = 2;
		nextBlockBi[nextBlockCount] = nextBi;
		nextBlockBj[nextBlockCount] = bj;
		nextBlockCount++;
	}

	if(nextBj >= 0 && nextBj < _heightfield->fieldData.numBlockZ) {
		int idx = nextBj * _heightfield->fieldData.numBlockX + bi;
		if(hfc.cacheTable[idx].flag > 0)
			hfc.cacheTable[idx].flag = 2;
		nextBlockBi[nextBlockCount] = bi;
		nextBlockBj[nextBlockCount] = nextBj;
		nextBlockCount++;
	}
	
	if((nextBi >= 0 && nextBi < _heightfield->fieldData.numBlockX) && (nextBj >= 0 && nextBj < _heightfield->fieldData.numBlockZ)) {
		int idx = nextBj * _heightfield->fieldData.numBlockX + nextBi;
		if(hfc.cacheTable[idx].flag > 0)
			hfc.cacheTable[idx].flag = 2;
		nextBlockBi[nextBlockCount] = nextBi;
		nextBlockBj[nextBlockCount] = nextBj;
		nextBlockCount++;
	}

	// キャッシュテーブルを更新
	int emptyCount = 0;
	for(int i=0;i<hfc.curBlockCount;i++) {
		int idx = hfc.curBlockBj[i] * _heightfield->fieldData.numBlockX + hfc.curBlockBi[i];
		if(hfc.cacheTable[idx].flag != 2) {
			emptyBuff[emptyCount++] = hfc.cacheTable[idx].idx;
			hfc.cacheTable[idx].flag = 0;
			hfc.cacheTable[idx].idx = 0;
		}
	}
	
	// バッファに読み込む
	int ibuff = 0;
	for(int i=0;i<nextBlockCount;i++) {
		int idx = nextBlockBj[i] * _heightfield->fieldData.numBlockX + nextBlockBi[i];
		if(hfc.cacheTable[idx].flag == 2) {
			hfc.cacheTable[idx].flag = 1;
		}
		else if(hfc.cacheTable[idx].flag == 0) {
			PFX_ASSERT(ibuff < emptyCount);
			
			// ロード

			memcpy(
				hfc.cacheData[emptyBuff[ibuff]],		// キャッシュブロック
				_heightfield->fieldData.blocks[idx],	// フィールド
				sizeof(HeightFieldBlock));				// サイズ


			hfc.cacheTable[idx].flag = 1;
			hfc.cacheTable[idx].idx = emptyBuff[ibuff];

			ibuff++;
		}
	}

	hfc.curBi = bi;
	hfc.curBj = bj;
	hfc.curBlockCount = nextBlockCount;
	__builtin_memcpy(hfc.curBlockBi,nextBlockBi,sizeof(int)*nextBlockCount);
	__builtin_memcpy(hfc.curBlockBj,nextBlockBj,sizeof(int)*nextBlockCount);
}

static float getCachedFieldData(int i,int j)
{
	int ci = i-hfc.curBi*BLOCK_SIZE;
	int cj = j-hfc.curBj*BLOCK_SIZE;
	int idx = hfc.curBj * _heightfield->fieldData.numBlockX + hfc.curBi;
	return hfc.cacheData[hfc.cacheTable[idx].idx].heightBuf[cj*BLOCK_SIZE_B+ci];
}

#endif

///////////////////////////////////////////////////////////////////////////////
// Ray Intersect

static inline bool intersectTriangle(Vector3 &p1,Vector3 &p2,Vector3 &p3,Vector3 &n,Vector3 &rayStart,Vector3 &rayDir,float &t)
{
	float v,w,d;
	Vector3 p1p2,p1p3;
	p1p2 = p2-p1;
	p1p3 = p3-p1;
	n = cross(p1p2,p1p3);
	
	d = dot(-rayDir,n);
	if(d <= 0.0f) return false;
	
	Vector3 pr = rayStart-p1;
	t = dot(pr,n);
	if(t < 0.0f || t > d) return false;
	
	Vector3 e = cross(-rayDir,pr);
	v = dot(p1p3,e);
	if(v < 0.0f || v > d) return false;

	w = -dot(p1p2,e);
	if(w < 0.0f || v+w > d) return false;
	
	t /= d;
	
	return true;
}

static bool setStartPosition(Vector3 &startPosL,Vector3 &endPosL,Vector3 &rayDirL)
{
	float tx=0.0f,tz=0.0f;
	
	// フィールドのX境界に接するtを決める
	if(startPosL[0] < 0.0f) {
		if(fabsf(rayDirL[0]) < HEPSILON) return false;
		tx = (0 - startPosL[0]) / rayDirL[0];
		if(tx < 0.0f || tx > 1.0f) return false;
	}
	else if(startPosL[0] > _heightfield->getFieldWidth()) {
		if(fabsf(rayDirL[0]) < HEPSILON) return false;
		tx = (_heightfield->getFieldWidth() - startPosL[0]) / rayDirL[0];
		if(tx < 0.0f || tx > 1.0f) return false;
	}
	
	// フィールドのZ境界に接するtを決める
	if(startPosL[2] < 0.0f) {
		if(fabsf(rayDirL[2]) < HEPSILON) return false;
		tz = (0 - startPosL[2]) / rayDirL[2];
		if(tz < 0.0f || tz > 1.0f) return false;
	}
	else if(startPosL[2] > _heightfield->getFieldDepth())
	{
		if(fabsf(rayDirL[2]) < HEPSILON) return false;
		tz = (_heightfield->getFieldDepth() - startPosL[2]) / rayDirL[2];
		if(tz < 0.0f || tz > 1.0f) return false;
	}
	
	// フィールドに接する座標を算出
	if(tx > tz) {
		startPosL = startPosL + tx * rayDirL;
		if(startPosL[2] < 0.0f || startPosL[2] > _heightfield->getFieldDepth())
			return false;
	}
	else {
		startPosL = startPosL + tz * rayDirL;
		if(startPosL[0] < 0.0f || startPosL[0] > _heightfield->getFieldWidth())
			return false;
	}
	
	rayDirL = endPosL-startPosL;
	
	return true;
}


#ifndef RAY_HEIGHTFIELD_CACHE_ENABLE

static bool detectIntersect(int xIdx,int zIdx,Vector3 &startInCell,Vector3 &endInCell,Vector3 &startPosL,Vector3 &endPosL,Vector3 &rayDirL,float &t,Vector3 &nml)
{
	(void) endPosL;

	// --------------------------------------------------
	// 高さ方向のチェック

	float h1 = getFieldData(_heightfield->fieldData, xIdx  , zIdx  );
	float h2 = getFieldData(_heightfield->fieldData, xIdx+1, zIdx  );
	float h3 = getFieldData(_heightfield->fieldData, xIdx  , zIdx+1);
	float h4 = getFieldData(_heightfield->fieldData, xIdx+1, zIdx+1);
	
	float hmin = PFX_MIN(PFX_MIN(PFX_MIN(h1,h2),h3),h4);
	float hmax = PFX_MAX(PFX_MAX(PFX_MAX(h1,h2),h3),h4);
	
	float rmin = PFX_MIN(startInCell[1],endInCell[1]);
	float rmax = PFX_MAX(startInCell[1],endInCell[1]);
	
	if(rmin > hmax || rmax < hmin)
		return false;
	
#ifdef CACHE_DEBUG_ENABLE
	fieldFlags[HFIELD_IDX(xIdx,zIdx)] = 2;
#endif

	// --------------------------------------------------
	// 面と線の衝突チェック
	{
		Vector3 p[4] = {
			Vector3(xIdx  ,h1,zIdx  ),
			Vector3(xIdx+1,h2,zIdx  ),
			Vector3(xIdx  ,h3,zIdx+1),
			Vector3(xIdx+1,h4,zIdx+1),
		};

		Vector3 n[2];
		float tt;
		
		// 面１
		if(intersectTriangle(p[0],p[2],p[1],n[0],startPosL,rayDirL,tt) && tt < t) {
			t = tt;

			Vector3 p1,p2,p3;
			p1 = mulPerElem(p[0],_heightfield->getScale());
			p2 = mulPerElem(p[2],_heightfield->getScale());
			p3 = mulPerElem(p[1],_heightfield->getScale());
			nml = normalize(cross(p2-p1,p3-p1));

#ifdef CACHE_DEBUG_ENABLE
	rayContactPoint = startPosL + tt * rayDirL;
#endif

			return true;
		}
		
		// 面２
		if(intersectTriangle(p[3],p[1],p[2],n[1],startPosL,rayDirL,tt) && tt < t) {
			t = tt;

			Vector3 p1,p2,p3;
			p1 = mulPerElem(p[3],_heightfield->getScale());
			p2 = mulPerElem(p[1],_heightfield->getScale());
			p3 = mulPerElem(p[2],_heightfield->getScale());
			nml = normalize(cross(p2-p1,p3-p1));

#ifdef CACHE_DEBUG_ENABLE
	rayContactPoint = startPosL + tt * rayDirL;
#endif

			return true;
		}
	}
	
	return false;
}

#else

static bool detectIntersect(int xIdx,int zIdx,Vector3 &startInCell,Vector3 &endInCell,Vector3 &startPosL,Vector3 &endPosL,Vector3 &rayDirL,float &t,Vector3 &nml,int stepX,int stepZ)
{
	(void) endPosL;

	// --------------------------------------------------
	// ブロックのキャッシュ
	
	int bi = xIdx/BLOCK_SIZE;
	int bj = zIdx/BLOCK_SIZE;
	
	if(hfc.firstBlock) {
		// ブロックを初期化
		initBlockData(bi,bj,stepX,stepZ);
		hfc.firstBlock = false;
	}
	else if(bi != hfc.curBi || bj != hfc.curBj) {
		// 前回の読込みのウェイト
		cellDmaWaitTagStatusAll(1<<1);

		// 参照先ブロックが変わったので、ブロックキャッシュを更新する
		// 次ブロックはあらかじめ先読みしているので、ここではウェイトしない
		updateBlockData(bi,bj,stepX,stepZ);
	}
	
	// --------------------------------------------------
	// 高さ方向のチェック

	float h1 = getCachedFieldData( xIdx  , zIdx  );
	float h2 = getCachedFieldData( xIdx+1, zIdx  );
	float h3 = getCachedFieldData( xIdx  , zIdx+1);
	float h4 = getCachedFieldData( xIdx+1, zIdx+1);
	
	float hmin = PFX_MIN(PFX_MIN(PFX_MIN(h1,h2),h3),h4);
	float hmax = PFX_MAX(PFX_MAX(PFX_MAX(h1,h2),h3),h4);
	
	float rmin = PFX_MIN(startInCell[1],endInCell[1]);
	float rmax = PFX_MAX(startInCell[1],endInCell[1]);
	
	if(rmin > hmax || rmax < hmin)
		return false;
	
#ifdef CACHE_DEBUG_ENABLE
	fieldFlags[HFIELD_IDX(xIdx,zIdx)] = 2;
#endif
	
	// --------------------------------------------------
	// 面と線の衝突チェック
	{
		Vector3 p[4] = {
			Vector3(xIdx  ,h1,zIdx  ),
			Vector3(xIdx+1,h2,zIdx  ),
			Vector3(xIdx  ,h3,zIdx+1),
			Vector3(xIdx+1,h4,zIdx+1),
		};

		Vector3 n[2];
		float tt;

		// 面１
		if(intersectTriangle(p[0],p[2],p[1],n[0],startPosL,rayDirL,tt) && tt < t) {
			t = tt;

			Vector3 p1,p2,p3;
			p1 = mulPerElem(p[0],_heightfield->getScale());
			p2 = mulPerElem(p[2],_heightfield->getScale());
			p3 = mulPerElem(p[1],_heightfield->getScale());
			nml = normalize(cross(p2-p1,p3-p1));

			return true;
		}
		
		// 面２
		if(intersectTriangle(p[3],p[1],p[2],n[1],startPosL,rayDirL,tt) && tt < t) {
			t = tt;

			Vector3 p1,p2,p3;
			p1 = mulPerElem(p[3],_heightfield->getScale());
			p2 = mulPerElem(p[1],_heightfield->getScale());
			p3 = mulPerElem(p[2],_heightfield->getScale());
			nml = normalize(cross(p2-p1,p3-p1));

			return true;
		}
	}
	
	return false;
}

#endif


bool rayIntersectHeightField(
	const CollPrim &prim,
	const Transform3 &transform,
	Ray &ray,float &t )
{

	_heightfield = const_cast<HeightField*>(prim.getHeightField());

	PFX_ASSERT(_heightfield);

#ifdef CACHE_DEBUG_ENABLE
	memset(fieldFlags,0,sizeof(uint32_t)*HEIGHTFIELD_X*HEIGHTFIELD_Z);
#endif

	Transform3 ti = orthoInverse(transform);

	Vector3 startPosL_org = _heightfield->worldToLocalPosition(ti.getTranslation()+ti*ray.startPos);
	Vector3 endPosL_org = _heightfield->worldToLocalPosition(ti.getTranslation()+ti*ray.endPos);
	Vector3 rayDirL_org = endPosL_org-startPosL_org;
	Vector3 startPosL = startPosL_org;
	Vector3 endPosL = endPosL_org;
	Vector3 rayDirL = endPosL-startPosL;

	int xIdx,zIdx;
	int xIdxLimit,zIdxLimit;
	int stepX,stepZ;
	float tMaxX,tMaxZ;
	float tDeltaX,tDeltaZ;
	float rayDirInvX,rayDirInvZ;
	
	// スタート地点を決める
	if(!setStartPosition(startPosL,endPosL,rayDirL)) {
#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE
		releaseHeightFieldCache();
#endif
		return false;
	}

	// ステップ用の変数を初期化

	xIdx = (int)floorf(startPosL[0]);
	zIdx = (int)floorf(startPosL[2]);

	stepX = rayDirL[0] > 0.0f ? 1 : -1;
	stepZ = rayDirL[2] > 0.0f ? 1 : -1;

	xIdxLimit = (int)floorf(endPosL[0])+stepX*2;
	zIdxLimit = (int)floorf(endPosL[2])+stepZ*2;

	rayDirInvX = fabsf(rayDirL[0]) < HEPSILON ? stepX*9999.0f : 1.0f/rayDirL[0];
	rayDirInvZ = fabsf(rayDirL[2]) < HEPSILON ? stepZ*9999.0f : 1.0f/rayDirL[2];

	tDeltaX = stepX*rayDirInvX;
	tDeltaZ = stepZ*rayDirInvZ;

	tMaxX = stepX>0 ? (floorf(startPosL[0]+1.0f)-startPosL[0])*rayDirInvX : (floorf(startPosL[0])-startPosL[0])*rayDirInvX;
	tMaxZ = stepZ>0 ? (floorf(startPosL[2]+1.0f)-startPosL[2])*rayDirInvZ : (floorf(startPosL[2])-startPosL[2])*rayDirInvZ;

#ifdef CACHE_DEBUG_ENABLE
	rayCellsCount = 0;
#endif

	Vector3 startInCell,endInCell;
	startInCell = startPosL;
	if(tMaxX < tMaxZ) {
		endInCell = startPosL + tMaxX * rayDirL;
	}
	else {
		endInCell = startPosL + tMaxZ * rayDirL;
	}

	bool ret =false;
	
	while(1) {
#ifdef CACHE_DEBUG_ENABLE
		fieldFlags[HFIELD_IDX(xIdx,zIdx)] = 1;
		rayCellsStart[rayCellsCount] = _heightfield->localToWorldPosition(startInCell);
		rayCellsEnd[rayCellsCount] = _heightfield->localToWorldPosition(endInCell);
		rayCellsCount++;
#endif

		// 面との衝突判定
		Vector3 nml;
#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE
		if(detectIntersect(xIdx,zIdx,startInCell,endInCell,startPosL_org,endPosL_org,rayDirL_org,t,nml,stepX,stepZ)) {
#else
		if(detectIntersect(xIdx,zIdx,startInCell,endInCell,startPosL_org,endPosL_org,rayDirL_org,t,nml)) {
#endif

#ifdef CACHE_DEBUG_ENABLE
			rayContactPoint = _heightfield->localToWorldPosition(rayContactPoint);
#endif
			ray.contactPoint = ray.startPos + t * ray.rayDir;
			ray.contactNormal = transform.getUpper3x3()*nml;
			ret = true;
			break;
		}

		// ステップを進める
		if(tMaxX < tMaxZ) {
			startInCell = startPosL + tMaxX * rayDirL;
			tMaxX+=tDeltaX;
			endInCell = startPosL + tMaxX * rayDirL;
			xIdx+=stepX;
			if(xIdx < 0 || xIdx >= _heightfield->getFieldWidth() || xIdx==xIdxLimit )
				break;
		}
		else {
			startInCell = startPosL + tMaxZ * rayDirL;
			tMaxZ+=tDeltaZ;
			endInCell = startPosL + tMaxZ * rayDirL;
			zIdx+=stepZ;
			if(zIdx < 0 || zIdx >= _heightfield->getFieldDepth() || zIdx==zIdxLimit )
				break;
		}
	}
	
#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE
	releaseHeightFieldCache();
#endif

	return ret;
	
}

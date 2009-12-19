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

#ifndef __SORT_COMMON_H__
#define __SORT_COMMON_H__

#include "Physics/Base/PhysicsCommon.h"

#define NULL_KEY	0xffffffff

#ifdef WIN32
	struct SortData {
		union {
			uint8_t   i8data[2][16];
			uint16_t  i16data[2][8];
			uint32_t  i32data[2][4];
		};
		
		SortData() {}
		SortData(uint32_t v)
		{
			i32data[0][0]=i32data[0][1]=i32data[0][2]=i32data[0][3]=v;
			i32data[1][0]=i32data[1][1]=i32data[1][2]=i32data[1][3]=v;
		}
	} __attribute__ ((aligned(16)));

	#define PFX_SET8(vecId,elem,data)  s.i8data[vecId][elem] = (uint8_t)data;
	#define PFX_SET16(vecId,elem,data) s.i16data[vecId][elem] = (uint16_t)data;
	#define PFX_SET32(vecId,elem,data) s.i32data[vecId][elem] = (uint32_t)data;
	#define PFX_GET8(vecId,elem)  s.i8data[vecId][elem];
	#define PFX_GET16(vecId,elem) s.i16data[vecId][elem];
	#define PFX_GET32(vecId,elem) s.i32data[vecId][elem];
#else
	#ifdef __PPU__
		#include <spu2vmx.h>
	#endif
	struct SortData {
		vec_uint4 vdata[2];

		SortData() {}
		SortData(uint32_t v)
		{
			vdata[0] = spu_splats(v);
			vdata[1] = spu_splats(v);
		}
	} __attribute__ ((aligned(16)));

	#define PFX_SET8(vecId,elem,data)  s.vdata[vecId]=(vec_uint4)spu_insert((uint8_t)data,(vec_uchar16)s.vdata[vecId],elem);
	#define PFX_SET16(vecId,elem,data) s.vdata[vecId]=(vec_uint4)spu_insert((uint16_t)data,(vec_ushort8)s.vdata[vecId],elem);
	#define PFX_SET32(vecId,elem,data) s.vdata[vecId]=(vec_uint4)spu_insert((uint32_t)data,(vec_uint4)s.vdata[vecId],elem);
	#define PFX_GET8(vecId,elem)  spu_extract((vec_uchar16)s.vdata[vecId],elem);
	#define PFX_GET16(vecId,elem) spu_extract((vec_ushort8)s.vdata[vecId],elem);
	#define PFX_GET32(vecId,elem) spu_extract((vec_uint4)s.vdata[vecId],elem);
#endif

// 検索キー
inline static void setKey(SortData& s,uint32_t key) {PFX_SET32(1,3,key);}
inline static uint32_t Key(const SortData& s) {return PFX_GET32(1,3);}

// ペア
inline static void setPair(SortData& s,uint32_t pair) {PFX_SET32(0,0,pair);}
inline static void setStateA(SortData& s,uint16_t i)	{PFX_SET16(0,2,i);}
inline static void setStateB(SortData& s,uint16_t i)	{PFX_SET16(0,3,i);}
inline static void setBodyA(SortData& s,uint16_t i)		{PFX_SET16(0,4,i);}
inline static void setBodyB(SortData& s,uint16_t i)		{PFX_SET16(0,5,i);}
inline static void setMovA(SortData& s,uint16_t i)		{PFX_SET16(0,6,i);}
inline static void setMovB(SortData& s,uint16_t i)		{PFX_SET16(0,7,i);}
inline static void setFlag(SortData& s,uint16_t flag)	{PFX_SET16(1,0,flag);}
inline static void setCallbackFlag(SortData& s,uint16_t flag)	{PFX_SET16(1,1,flag);}

inline static uint32_t Pair(const SortData& s)   {return PFX_GET32(0,0);}
inline static uint16_t StateA(const SortData& s) {return PFX_GET16(0,2);}
inline static uint16_t StateB(const SortData& s) {return PFX_GET16(0,3);}
inline static uint16_t BodyA(const SortData& s)  {return PFX_GET16(0,4);}
inline static uint16_t BodyB(const SortData& s)  {return PFX_GET16(0,5);}
inline static uint16_t MovA(const SortData& s)   {return PFX_GET16(0,6);}
inline static uint16_t MovB(const SortData& s)   {return PFX_GET16(0,7);}
inline static uint16_t Flag(const SortData& s)   {return PFX_GET16(1,0);}
inline static uint16_t CallbackFlag(const SortData& s)	{return PFX_GET16(1,1);}

// AABB
inline static void setXMin(SortData& s,uint16_t i) {PFX_SET16(0,0,i);}
inline static void setXMax(SortData& s,uint16_t i) {PFX_SET16(0,1,i);}
inline static void setYMin(SortData& s,uint16_t i) {PFX_SET16(0,2,i);}
inline static void setYMax(SortData& s,uint16_t i) {PFX_SET16(0,3,i);}
inline static void setZMin(SortData& s,uint16_t i) {PFX_SET16(0,4,i);}
inline static void setZMax(SortData& s,uint16_t i) {PFX_SET16(0,5,i);}
inline static void setXYZMin(SortData& s,uint16_t i,int axis) {PFX_SET16(0,axis<<1,i);}
inline static void setXYZMax(SortData& s,uint16_t i,int axis) {PFX_SET16(0,1+(axis<<1),i);}

inline static uint16_t XMin(const SortData& s) {return PFX_GET16(0,0);}
inline static uint16_t XMax(const SortData& s) {return PFX_GET16(0,1);}
inline static uint16_t YMin(const SortData& s) {return PFX_GET16(0,2);}
inline static uint16_t YMax(const SortData& s) {return PFX_GET16(0,3);}
inline static uint16_t ZMin(const SortData& s) {return PFX_GET16(0,4);}
inline static uint16_t ZMax(const SortData& s) {return PFX_GET16(0,5);}
inline static uint16_t XYZMin(const SortData& s,int axis) {return PFX_GET16(0,axis<<1);}
inline static uint16_t XYZMax(const SortData& s,int axis) {return PFX_GET16(0,1+(axis<<1));}

inline static void setStateId(SortData& s,uint16_t i) {PFX_SET16(0,6,i);}
inline static void setBodyId(SortData& s,uint16_t i)  {PFX_SET16(0,7,i);}
inline static void setMovType(SortData& s,uint16_t i) {PFX_SET16(1,0,i);}
inline static void setCallback(SortData& s,uint16_t i){PFX_SET16(1,1,i);}
inline static void setSelf(SortData& s,uint32_t i)    {PFX_SET32(1,1,i);}
inline static void setTarget(SortData& s,uint32_t i)  {PFX_SET32(1,2,i);}

inline static uint16_t StateId(const SortData& s) {return PFX_GET16(0,6);}
inline static uint16_t BodyId(const SortData& s)  {return PFX_GET16(0,7);}
inline static uint16_t MovType(const SortData& s) {return PFX_GET16(1,0);}
inline static uint16_t Callback(const SortData& s){return PFX_GET16(1,1);}
inline static uint32_t Self(const SortData& s)    {return PFX_GET32(1,1);}
inline static uint32_t Target(const SortData& s)  {return PFX_GET32(1,2);}

inline static void setStatePair(SortData& s,uint16_t i,uint16_t j)
{
	uint32_t minIdx = i < j ? i : j;
	uint32_t maxIdx = i > j ? i : j;
	setKey(s,maxIdx * (maxIdx - 1) / 2 + minIdx);
	setStateA(s,i);
	setStateB(s,j);
}

inline static void setStatePair2(SortData& s,uint16_t i,uint16_t j)
{
	// i==jがあり得る場合はこちらを使用する
	uint32_t minIdx = i < j ? i : j;
	uint32_t maxIdx = i > j ? i : j;
	setKey(s,maxIdx * (maxIdx + 1) / 2 + minIdx);
	setStateA(s,i);
	setStateB(s,j);
}

#endif

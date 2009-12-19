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

#ifndef __PFX_AABB16_H__
#define __PFX_AABB16_H__

#include "Physics/Base/PhysicsCommon.h"



struct PfxAABB16
{
	uint32_t data[4];
} __attribute__ ((aligned(16)));

inline static void setXMin(PfxAABB16& s,uint16_t i) {s.data[0]&=0x0000ffff;s.data[0]|=i << 16;}
inline static void setXMax(PfxAABB16& s,uint16_t i) {s.data[0]&=0xffff0000;s.data[0]|=i;}
inline static void setYMin(PfxAABB16& s,uint16_t i) {s.data[1]&=0x0000ffff;s.data[1]|=i << 16;}
inline static void setYMax(PfxAABB16& s,uint16_t i) {s.data[1]&=0xffff0000;s.data[1]|=i;}
inline static void setZMin(PfxAABB16& s,uint16_t i) {s.data[2]&=0x0000ffff;s.data[2]|=i << 16;}
inline static void setZMax(PfxAABB16& s,uint16_t i) {s.data[2]&=0xffff0000;s.data[2]|=i;}
inline static void setData16_0(PfxAABB16& s,uint16_t i) {s.data[3]&=0x0000ffff;s.data[3]|=i << 16;}
inline static void setData16_1(PfxAABB16& s,uint16_t i) {s.data[3]&=0xffff0000;s.data[3]|=i;}
inline static void setData32(PfxAABB16& s,uint32_t i) {s.data[3]=i;}

inline static uint16_t XMin(const PfxAABB16& s) {return ((uint16_t)(s.data[0] >> 16));}
inline static uint16_t XMax(const PfxAABB16& s) {return ((uint16_t)(s.data[0] & 0x0000ffff));}
inline static uint16_t YMin(const PfxAABB16& s) {return ((uint16_t)(s.data[1] >> 16));}
inline static uint16_t YMax(const PfxAABB16& s) {return ((uint16_t)(s.data[1] & 0x0000ffff));}
inline static uint16_t ZMin(const PfxAABB16& s) {return ((uint16_t)(s.data[2] >> 16));}
inline static uint16_t ZMax(const PfxAABB16& s) {return ((uint16_t)(s.data[2] & 0x0000ffff));}
inline static uint16_t Data16_0(const PfxAABB16& s) {return ((uint16_t)(s.data[3] >> 16));}
inline static uint16_t Data16_1(const PfxAABB16& s) {return ((uint16_t)(s.data[3] & 0x0000ffff));}
inline static uint32_t Data32(const PfxAABB16& s) {return s.data[3];}

inline bool testAABB(const PfxAABB16 &aabbA,const PfxAABB16 &aabbB)
{
	if(XMax(aabbA) < XMin(aabbB) || XMin(aabbA) > XMax(aabbB)) return false;
	if(YMax(aabbA) < YMin(aabbB) || YMin(aabbA) > YMax(aabbB)) return false;
	if(ZMax(aabbA) < ZMin(aabbB) || ZMin(aabbA) > ZMax(aabbB)) return false;
	return true;
}

inline PfxAABB16 mergeAABB(const PfxAABB16 &aabbA,const PfxAABB16 &aabbB)
{
	PfxAABB16 aabb;
	setXMin(aabb,PFX_MIN(XMin(aabbA),XMin(aabbB)));
	setXMax(aabb,PFX_MAX(XMax(aabbA),XMax(aabbB)));
	setYMin(aabb,PFX_MIN(YMin(aabbA),YMin(aabbB)));
	setYMax(aabb,PFX_MAX(YMax(aabbA),YMax(aabbB)));
	setZMin(aabb,PFX_MIN(ZMin(aabbA),ZMin(aabbB)));
	setZMax(aabb,PFX_MAX(ZMax(aabbA),ZMax(aabbB)));
	return aabb;
}



#endif /* __PFX_AABB16_H__ */

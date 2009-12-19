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
#ifndef __TESTAABB_H__
#define __TESTAABB_H__

///////////////////////////////////////////////////////////////////////////////
// AABB

inline bool testAABB(const TrbState &stateA,const TrbState &stateB)
{
	if(fabs(stateA.center[0]-stateB.center[0]) > (stateA.half[0]+stateB.half[0])) return false;
	if(fabs(stateA.center[1]-stateB.center[1]) > (stateA.half[1]+stateB.half[1])) return false;
	if(fabs(stateA.center[2]-stateB.center[2]) > (stateA.half[2]+stateB.half[2])) return false;
	return true;
}

inline bool testAABB16(const SortData &aabbA,const SortData &aabbB)
{
//#ifdef TRY_SIMD
	if(XMax(aabbA) < XMin(aabbB) || XMin(aabbA) > XMax(aabbB)) return false;
	if(YMax(aabbA) < YMin(aabbB) || YMin(aabbA) > YMax(aabbB)) return false;
	if(ZMax(aabbA) < ZMin(aabbB) || ZMin(aabbA) > ZMax(aabbB)) return false;
	return true;
}

#endif /* __TESTAABB_H__ */

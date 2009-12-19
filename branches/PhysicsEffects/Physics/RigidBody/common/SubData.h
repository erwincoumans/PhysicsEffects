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

#ifndef __SUBDATA_H__
#define __SUBDATA_H__

struct SubData {
	enum {
		SubDataNone = 0,
		SubDataFacetLocal,
	};
	
	uint8_t  type;
	
	// 面ローカル座標
	struct {
		uint8_t  islandIdx;	// アイランドインデックス
		uint8_t  facetIdx; 	// 面インデックス
		uint16_t s;        	// 面ローカル座標系の衝突点を表すs,t(s>=0,t>=0,s+t<=1)
		uint16_t t;        	// 座標 = s * (p1-p0) + t * (p2-p0)
	} facetLocal;
	
	SubData()
	{
		type = 0;
		facetLocal.islandIdx = 0;
		facetLocal.facetIdx = 0;
		facetLocal.s = 0;
		facetLocal.t = 0;
	}

	void  setIslandIndex(uint8_t i) {facetLocal.islandIdx = i;}
	void  setFacetIndex(uint8_t i) {facetLocal.facetIdx = i;}
	void  setFacetLocalS(float s) {facetLocal.s = (uint16_t)(s * 65535.0f);}
	void  setFacetLocalT(float t) {facetLocal.t = (uint16_t)(t * 65535.0f);}

	uint8_t getIslandIndex() {return facetLocal.islandIdx;}
	uint8_t getFacetIndex() {return facetLocal.facetIdx;}
	float getFacetLocalS() {return facetLocal.s / 65535.0f;}
	float getFacetLocalT() {return facetLocal.t / 65535.0f;}
};

#endif

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
#ifndef __PARALLEL_GROUP_H__
#define __PARALLEL_GROUP_H__

///////////////////////////////////////////////////////////////////////////////
// Parallel Pair Group

#define MAX_SOLVER_PHASES 64	// 最大フェーズ数
#define MAX_SOLVER_GROUPS 16	// １フェーズに含まれる最大並列処理グループ
#define MAX_SOLVER_PAIRS  128	// １グループに含まれる最大ペア数
#define MIN_SOLVER_PAIRS  16

struct SolverGroup {
	uint16_t pairIndices[MAX_SOLVER_PAIRS];
} __attribute__ ((aligned(128)));

struct SolverInfo {
	uint16_t numPhases;
	uint16_t numGroups[MAX_SOLVER_PHASES]; // 各フェーズの保持する並列実行可能なグループ数
	uint16_t numPairs[MAX_SOLVER_PHASES*MAX_SOLVER_GROUPS]; // 各グループの保持するペア数
} __attribute__ ((aligned(16)));

#endif /* __PARALLEL_GROUP_H__ */

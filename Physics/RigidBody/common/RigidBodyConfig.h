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

#ifndef __RIGIDBODY_CONFIG_H__
#define __RIGIDBODY_CONFIG_H__

//---------------------------------------------------------------------------
// Rigid Body Dynamics
//---------------------------------------------------------------------------

// Rigid Body
#define NUMPRIMS					64		// １剛体に含まれる形状数

// Simulation Settings
#define JOINT_IMPULSE_EPSILON		0.0001f	// インパルス閾値
#define NUMCONTACTS_PER_BODIES		4		// 剛体ペアの最大衝突点数
#define CONTACT_THRESHOLD_NORMAL	0.01f	// 衝突点の閾値（法線方向）
#define CONTACT_THRESHOLD_TANGENT	0.002f	// 衝突点の閾値（平面上）
#define CONTACT_BATCH				64		// 衝突ペア処理数の最大値
#define CCD_THRESHOLD_MIN			0.01f	// CCD判定の範囲
#define CCD_THRESHOLD_MAX			0.25f	// CCD判定の範囲
#define CCD_ENABLE_DISTANCE			0.5f	// CCD判定に移行するための間隔
#define CONTACT_SLOP				0.001f	// 衝突貫通深度の許容値
#define JOINT_LIN_SLOP				0.01f	// ジョイント位置エラーの許容値
#define JOINT_ANG_SLOP				0.01f	// ジョイント角度エラーの許容値

// 積分計算の切り替え（どちらかを選択）
//#define ODE_EULER
#define ODE_RUNGEKUTTA

// Height Field
#define BLOCK_SIZE				16
#define HEIGHTFIELD_CACHE_COUNT 4
#define BLOCK_SIZE_B (BLOCK_SIZE+1)	// 実際に確保されるサイズ（エッジを含む）

// SPU-SIMD最適化
#ifdef __SPU__
	//#define TRY_SIMD
#endif

#endif /* __RIGIDBODY_CONFIG_H__ */

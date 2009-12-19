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

#ifndef __HEIGHTFIELD_FUNCTION_H__
#define __HEIGHTFIELD_FUNCTION_H__

#include "Physics/Base/PhysicsCommon.h"

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "Physics/RigidBody/common/HeightField.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"

///////////////////////////////////////////////////////////////////////////////
// Height Field Function


// 指定したハイトフィールドの高さ（ローカル）を取得する
// ※ x,zはハイトフィールドのローカル座標で指定する
bool getHeight(const HeightField &heightfield,float x,float z,float &h);

// 指定したハイトフィールドの法線（ワールド）を取得する
// ※ x,zはハイトフィールドのローカル座標で指定する
bool getNormal(const HeightField &heightfield,float x,float z,Vector3 &nml);

#endif

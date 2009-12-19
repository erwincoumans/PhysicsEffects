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

#ifndef __PFX_COMMON_TYPES_H__
#define __PFX_COMMON_TYPES_H__

typedef bool			PfxBool;
typedef char			PfxInt8;
typedef short			PfxInt16;
typedef long			PfxInt32;
typedef unsigned char	PfxUInt8;
typedef unsigned short	PfxUInt16;
typedef unsigned long	PfxUInt32;
typedef float			PfxFloat;

struct PfxFloat3 {
	PfxFloat v[3];
	
	PfxFloat3() {}

	PfxFloat3(PfxFloat f)
	{
		v[0] = v[1] = v[2] = f;
	}

	PfxFloat3(PfxFloat f0,PfxFloat f1,PfxFloat f2)
	{
		v[0] = f0;
		v[1] = f1;
		v[2] = f2;
	}

	PfxFloat& operator[](int idx)
	{
		return v[idx];
	}

	PfxFloat operator[](int idx) const
	{
		return v[idx];
	}
};

struct PfxFloat4 {
	PfxFloat v[4];

	PfxFloat4() {}

	PfxFloat4(PfxFloat f)
	{
		v[0] = v[1] = v[2] = v[3] = f;
	}

	PfxFloat4(PfxFloat f0,PfxFloat f1,PfxFloat f2,PfxFloat f3)
	{
		v[0] = f0;
		v[1] = f1;
		v[2] = f2;
		v[3] = f3;
	}
	
	PfxFloat& operator[](int idx)
	{
		return v[idx];
	}

	PfxFloat operator[](int idx) const
	{
		return v[idx];
	}
};

struct PfxMatrix3 {
	PfxFloat3 m[3];
	
	PfxMatrix3() {}
	
	PfxMatrix3(
		PfxFloat f0,PfxFloat f1,PfxFloat f2,
		PfxFloat f3,PfxFloat f4,PfxFloat f5,
		PfxFloat f6,PfxFloat f7,PfxFloat f8)
	{
		m[0][0]=f0;m[0][1]=f1;m[0][2]=f2;
		m[1][0]=f3;m[1][1]=f4;m[1][2]=f5;
		m[2][0]=f6;m[2][1]=f7;m[2][2]=f8;
	}
	
	PfxMatrix3(PfxFloat f)
	{
		m[0][0]=m[0][1]=m[0][2]=f;
		m[1][0]=m[1][1]=m[1][2]=f;
		m[2][0]=m[2][1]=m[2][2]=f;
	}
	
	void identity() {
		*this = PfxMatrix3(
			1.0f,0.0f,0.0f,
			0.0f,1.0f,0.0f,
			0.0f,0.0f,1.0f);
	}

	PfxFloat3& operator[](int idx)
	{
		return m[idx];
	}

	PfxFloat3 operator[](int idx) const
	{
		return m[idx];
	}
};

#endif /* __PFX_COMMON_TYPES_H__ */

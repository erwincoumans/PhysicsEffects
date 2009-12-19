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

#ifndef __VECINT3_H__
#define __VECINT3_H__

#include <vectormath_aos.h>
using namespace Vectormath::Aos;



class VecInt3
{
private:
	int32_t mX,mY,mZ,mW;

public:
	VecInt3() {mX=mY=mZ=mW=0;}
	VecInt3(const Vector3 vec) {mX=(int32_t)vec[0];mY=(int32_t)vec[1];mZ=(int32_t)vec[2];mW=0;}
	VecInt3(float fx,float fy,float fz) {mX=(int32_t)fx;mY=(int32_t)fy;mZ=(int32_t)fz;mW=0;}
	VecInt3(int32_t iv) {mX=mY=mZ=iv;mW=0;}
	VecInt3(int32_t ix,int32_t iy,int32_t iz) {mX=ix;mY=iy;mZ=iz;mW=0;}

    inline VecInt3 &operator =( const VecInt3 &vec);

	inline int32_t get(int32_t i) const {return *(&mX+i);}
	inline int32_t getX() const {return mX;}
	inline int32_t getY() const {return mY;}
	inline int32_t getZ() const {return mZ;}
	inline void set(int32_t i,int32_t v) {*(&mX+i) = v;}
	inline void setX(int32_t v) {mX = v;}
	inline void setY(int32_t v) {mY = v;}
	inline void setZ(int32_t v) {mZ = v;}

    inline const VecInt3 operator +( const VecInt3 & vec ) const;
    inline const VecInt3 operator -( const VecInt3 & vec ) const;
    inline const VecInt3 operator *( int32_t scalar ) const;
    inline const VecInt3 operator /( int32_t scalar ) const;

    inline VecInt3 & operator +=( const VecInt3 & vec );
    inline VecInt3 & operator -=( const VecInt3 & vec );
    inline VecInt3 & operator *=( int32_t scalar );
    inline VecInt3 & operator /=( int32_t scalar );

    inline const VecInt3 operator -() const;

	operator Vector3() const
	{
		return Vector3((float)mX,(float)mY,(float)mZ);
	}
} __attribute__ ((aligned(16)));

inline VecInt3 &VecInt3::operator =( const VecInt3 &vec)
{
    mX = vec.mX;
    mY = vec.mY;
    mZ = vec.mZ;
    return *this;
}

inline const VecInt3 VecInt3::operator +( const VecInt3 & vec ) const
{
    return VecInt3(mX+vec.mX, mY+vec.mY, mZ+vec.mZ);
}

inline const VecInt3 VecInt3::operator -( const VecInt3 & vec ) const
{
    return VecInt3(mX-vec.mX, mY-vec.mY, mZ-vec.mZ);
}

inline const VecInt3 VecInt3::operator *( int32_t scalar ) const
{
    return VecInt3(mX*scalar, mY*scalar, mZ*scalar);
}

inline const VecInt3 VecInt3::operator /( int32_t scalar ) const
{
    return VecInt3(mX/scalar, mY/scalar, mZ/scalar);
}

inline VecInt3 &VecInt3::operator +=( const VecInt3 & vec )
{
    *this = *this + vec;
    return *this;
}

inline VecInt3 &VecInt3::operator -=( const VecInt3 & vec )
{
    *this = *this - vec;
    return *this;
}

inline VecInt3 &VecInt3::operator *=( int32_t scalar )
{
    *this = *this * scalar;
    return *this;
}

inline VecInt3 &VecInt3::operator /=( int32_t scalar )
{
    *this = *this / scalar;
    return *this;
}

inline const VecInt3 VecInt3::operator -() const
{
	return VecInt3(-mX,-mY,-mZ);
}

inline const VecInt3 operator *( int32_t scalar, const VecInt3 & vec )
{
    return vec * scalar;
}

inline const VecInt3 mulPerElem( const VecInt3 & vec0, const VecInt3 & vec1 )
{
	return VecInt3(vec0.getX()*vec1.getX(), vec0.getY()*vec1.getY(), vec0.getZ()*vec1.getZ());
}

inline const VecInt3 divPerElem( const VecInt3 & vec0, const VecInt3 & vec1 )
{
	return VecInt3(vec0.getX()/vec1.getX(), vec0.getY()/vec1.getY(), vec0.getZ()/vec1.getZ());
}

inline const VecInt3 absPerElem( const VecInt3 & vec )
{
	return VecInt3(abs(vec.getX()), abs(vec.getY()), abs(vec.getZ()));
}

inline const VecInt3 maxPerElem( const VecInt3 & vec0, const VecInt3 & vec1 )
{
    return VecInt3(
        (vec0.getX() > vec1.getX())? vec0.getX() : vec1.getX(),
        (vec0.getY() > vec1.getY())? vec0.getY() : vec1.getY(),
        (vec0.getZ() > vec1.getZ())? vec0.getZ() : vec1.getZ()
    );
}

inline const VecInt3 minPerElem( const VecInt3 & vec0, const VecInt3 & vec1 )
{
    return VecInt3(
        (vec0.getX() < vec1.getX())? vec0.getX() : vec1.getX(),
        (vec0.getY() < vec1.getY())? vec0.getY() : vec1.getY(),
        (vec0.getZ() < vec1.getZ())? vec0.getZ() : vec1.getZ()
    );
}



#endif /* __VECINT3_H__ */

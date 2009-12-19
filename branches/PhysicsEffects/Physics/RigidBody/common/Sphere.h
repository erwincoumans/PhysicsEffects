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

#ifndef __SPHERE_H__
#define __SPHERE_H__

#include "Box.h"

//---------------------------------------------------------------------------
// Sphere
//---------------------------------------------------------------------------

class Sphere
{
public:
	float radius;

	Sphere()
	{}
	Sphere( float _radius );

	void  Set( float _radius );
	Vector3   GetAABB();
};

inline
Sphere::Sphere( float _radius )
{
	radius = _radius;
}

inline
void
Sphere::Set( float _radius )
{
	radius = _radius;
}

inline
Vector3
Sphere::GetAABB()
{
	return Vector3( radius );
}

//-------------------------------------------------------------------------------------------------
// SpherePoint
//-------------------------------------------------------------------------------------------------

class SpherePoint
{
public:
	SpherePoint() : localPoint(0.0f) {}

	Point3      localPoint;    // point in sphere's coordinate system
	FeatureType featureType;
	int         featureIdx;

	inline void setVertexFeature();
};

inline
void
SpherePoint::setVertexFeature()
{
	featureType = V;
	featureIdx = 0;
}

#endif /* __SPHERE_H__ */


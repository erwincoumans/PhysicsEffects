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

#ifndef __CAPSULE_H__
#define __CAPSULE_H__

#include "Box.h"

//---------------------------------------------------------------------------
// Capsule
//---------------------------------------------------------------------------

class Capsule
{
public:
	float hLength;
	float radius;

	Capsule()
	{}
	Capsule(float hLength_, float radius_);

	void Set(float hLength_, float radius_);
	Vector3  GetAABB(Vector3 direction);
};

inline
Capsule::Capsule(float hLength_, float radius_)
{
	hLength = hLength_;
	radius = radius_;
}

inline
void
Capsule::Set(float hLength_, float radius_)
{
	hLength = hLength_;
	radius = radius_;
}

inline
Vector3
Capsule::GetAABB(Vector3 direction)
{
	return absPerElem(direction) * hLength + Vector3(radius);
}

//-------------------------------------------------------------------------------------------------
// CapsulePoint
//-------------------------------------------------------------------------------------------------

class CapsulePoint
{
public:
	CapsulePoint() : localPoint(0.0f) {}

	Point3      localPoint;    // point in capsule's coordinate system
	FeatureType featureType;
	int         featureIdx;
	float       lineParam;     // if line segment inside the capsule is parameterized as
	//    (center + direction * t), this is the t value for the
	//    nearest point on the line segment.

	inline void setVertexFeature(int plus);
	inline void setEdgeFeature();

	inline void getVertexFeature(int & plus) const;
};

inline
void
CapsulePoint::setVertexFeature(int plus)
{
	featureType = V;
	featureIdx = plus;
}

inline
void
CapsulePoint::setEdgeFeature()
{
	featureType = E;
	featureIdx = 0;
}

inline
void
CapsulePoint::getVertexFeature(int & plus) const
{
	plus = featureIdx;
}

#endif /* __CAPSULE_H__ */


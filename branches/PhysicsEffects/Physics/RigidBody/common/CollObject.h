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

#ifndef __COLLOBJECT_H__
#define __COLLOBJECT_H__

#include "CollPrim.h"
#include "Physics/RigidBody/common/RigidBodyConfig.h"

///////////////////////////////////////////////////////////////////////////////
// Collidable Object

class CollObject
{
friend class RigidBodies;
friend class PrimIterator;

private:
	CollPrim *mPrimBase;
	uint16_t mPrimIds[NUMPRIMS];
	uint8_t mNumPrims;
	uint8_t mMaxPrims;

	float mCcdRadius;
	float mCenter[3];	// AABB center (Local)
	float mHalf[3];		// AABB half (Local)

	CollPrim mDefPrim;
	
	CollPrim &getNewPrim()
	{
		PFX_ASSERT(mNumPrims<=mMaxPrims);
		if(mNumPrims == 0) {
			mNumPrims++;
			return mDefPrim;
		}
		else {
			mNumPrims++;
			return mPrimBase[mPrimIds[mNumPrims-2]];
		}
	}
	
public:

	CollObject();
	~CollObject();

	void addBox(Box box, const Transform3 & objectRelTransform);
	void addCapsule(Capsule capsule, const Transform3 & objectRelTransform);
	void addSphere(Sphere sphere, const Transform3 & objectRelTransform);
	void addConvex(const ConvexMesh *convexMesh, const Transform3 & objectRelTransform);
	void setHeightField(const HeightField *heightfield);
	void setLargeMesh(const LargeTriMesh *largeMesh);
	void setPreLargeMesh(const LargeTriMesh *largeMesh);

	void finish();
	void clear(); // clear all the prims

	inline uint8_t getNumPrims() const;
	const CollPrim& getDefPrim() const {return mDefPrim;}
	CollPrim& getDefPrim() {return mDefPrim;}

#ifndef __SPU__
	inline const CollPrim& getPrim(int i) const;
	inline CollPrim& getPrim(int i);
#endif

	inline float getRadius() const;
	inline Vector3 getHalf() const;
	inline Vector3 getCenter() const;
	
	inline void setCcdRadius(float r) {mCcdRadius=r;}
	inline float getCcdRadius() const {return mCcdRadius;}
	
} __attribute__ ((aligned(128)));

inline uint8_t CollObject::getNumPrims() const
{
	return mNumPrims;
}

#ifndef __SPU__
inline const CollPrim& CollObject::getPrim(int i) const
{
	PFX_ASSERT(i<mNumPrims);
	if(i>0) {
		return mPrimBase[mPrimIds[i-1]];
	}
	else {
		return mDefPrim;
	}
}

inline CollPrim& CollObject::getPrim(int i)
{
	PFX_ASSERT(i<mNumPrims);
	if(i>0) {
		return mPrimBase[mPrimIds[i-1]];
	}
	else {
		return mDefPrim;
	}
}
#endif

inline float CollObject::getRadius() const
{
	return length(getHalf());
}

inline Vector3 CollObject::getHalf() const
{
	return read_Vector3(mHalf);
}

inline Vector3 CollObject::getCenter() const
{
	return read_Vector3(mCenter);
}

inline void CollObject::setPreLargeMesh(const LargeTriMesh *largeMesh)
{
	if(mDefPrim.getType() == LARGEMESH)
		mDefPrim.setPreLargeMesh(largeMesh);
}

class PrimIterator
{
private:
	uint32_t mNumPrims;
	CollPrim *mPrimBase;
	const uint16_t *mPrimIds;
	const CollPrim *mCurPrim;
	uint32_t mIndex;

public:
	PrimIterator(const CollObject &coll) : mPrimIds(coll.mPrimIds)
	{
		mNumPrims = coll.mNumPrims;
		mPrimBase = coll.mPrimBase;
		mIndex = 0;
		mCurPrim = &coll.mDefPrim;
	}

	~PrimIterator() {}

	inline PrimIterator& operator++()
	{
		mCurPrim = &mPrimBase[mPrimIds[mIndex++]];
		return *this;
	}

	const CollPrim& operator*() const {return *mCurPrim;}
};

#endif /* __COLLOBJECT_H__ */


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

#include <float.h>
#include <string.h>
#include "Physics/RigidBody/common/CollObject.h"
#include "Physics/RigidBody/common/HeightField.h"
#include "TriMesh.h"

CollObject::CollObject()
{
	clear();
}

CollObject::~CollObject()
{
}

void
CollObject::clear()
{
	mNumPrims = 0;
	mMaxPrims = 1;
	mCenter[0] = 0.0f;
	mCenter[1] = 0.0f;
	mCenter[2] = 0.0f;
	mHalf[0] = 0.0f;
	mHalf[1] = 0.0f;
	mHalf[2] = 0.0f;
	mCcdRadius = 0.0f;
}

void
CollObject::addBox( Box box, const Transform3 & objectRelTransform )
{
	if ( mNumPrims < mMaxPrims ) {
		CollPrim &newPrim = getNewPrim();
		newPrim.setBox( box );
		newPrim.setObjectRelTransform( objectRelTransform );
	}
}

void
CollObject::addCapsule( Capsule capsule, const Transform3 & objectRelTransform )
{
	if ( mNumPrims < mMaxPrims ) {
		CollPrim &newPrim = getNewPrim();
		newPrim.setCapsule( capsule );
		newPrim.setObjectRelTransform( objectRelTransform );
	}
}

void
CollObject::addSphere( Sphere sphere, const Transform3 & objectRelTransform )
{
	if ( mNumPrims < mMaxPrims ) {
		CollPrim &newPrim = getNewPrim();
		newPrim.setSphere( sphere );
		newPrim.setObjectRelTransform( objectRelTransform );
	}
}

void
CollObject::addConvex( const ConvexMesh *convexMesh, const Transform3 & objectRelTransform )
{
	PFX_ASSERT(convexMesh);

	if ( mNumPrims < mMaxPrims ) {
		CollPrim &newPrim = getNewPrim();
		newPrim.setConvexMesh( convexMesh );
		newPrim.setObjectRelTransform( objectRelTransform );
	}
}

void
CollObject::finish()
{
#ifndef __SPU__
	if( mNumPrims == 0 || mDefPrim.getType() == HEIGHTFIELD || mDefPrim.getType() == LARGEMESH )
		return;

	// compute AABB
	Vector3 vcenter(0.0f),vhalf(0.0f);
	mCcdRadius = 0.0f;

	Vector3 halfMax(-FLT_MAX),halfMin(FLT_MAX);

	PrimIterator itrPrim(*this);
	for (uint32_t i=0;i<getNumPrims();i++,++itrPrim) {
		const CollPrim &prim = *itrPrim;
		Vector3 primHalf(0.0f);
		
		if ( prim.getType() == BOX ) {
			primHalf = prim.getBox().GetAABB(prim.getObjectRelTransform().getUpper3x3());
		} else if ( prim.getType() == CAPSULE ) {
			primHalf = prim.getCapsule().GetAABB(prim.getObjectRelTransform().getCol(0));
		} else if ( prim.getType() == SPHERE ) {
			primHalf = prim.getSphere().GetAABB();
		} else if ( prim.getType() == CONVEXMESH ) {
			primHalf = prim.getConvexMesh()->getAABB(prim.getObjectRelTransform().getUpper3x3());
		}

		Vector3 relPos = prim.getObjectRelTransform().getTranslation();

		halfMax = maxPerElem(halfMax,relPos + primHalf);
		halfMin = minPerElem(halfMin,relPos - primHalf);
	}
	
	vcenter = ( halfMin + halfMax ) * 0.5f;
	vhalf = ( halfMax - halfMin ) * 0.5f;
	mCenter[0] = vcenter[0];
	mCenter[1] = vcenter[1];
	mCenter[2] = vcenter[2];
	mHalf[0] = vhalf[0];
	mHalf[1] = vhalf[1];
	mHalf[2] = vhalf[2];
	mCcdRadius = length(vhalf);
#endif
}

void
CollObject::setHeightField(const HeightField *heightfield)
{
	PFX_ASSERT(heightfield);

	mDefPrim.setHeightField(heightfield);
	mDefPrim.setObjectRelTransform(Transform3::identity());
	mNumPrims=1;

	Vector3 vhalf(0.0f),vcenter(0.0f);

	vhalf =	0.5f*Vector3(
				heightfield->getFieldWidth(),
				(heightfield->getMaxHeight()-heightfield->getMinHeight()),
				heightfield->getFieldDepth() );

	vhalf =  mulPerElem(heightfield->getScale(),vhalf);
	vcenter = Vector3(0.0f,0.5f*heightfield->getScale()[1]*(heightfield->getMaxHeight()+heightfield->getMinHeight()),0.0f);
	mCenter[0] = vcenter[0];
	mCenter[1] = vcenter[1];
	mCenter[2] = vcenter[2];
	mHalf[0] = vhalf[0];
	mHalf[1] = vhalf[1];
	mHalf[2] = vhalf[2];
	mCcdRadius = 0.0f;
}

void
CollObject::setLargeMesh(const LargeTriMesh *largeMesh)
{
	PFX_ASSERT(largeMesh);

	mDefPrim.setLargeMesh(largeMesh);
	mDefPrim.setObjectRelTransform(Transform3::identity());
	mDefPrim.viData[1] = mDefPrim.viData[2] = 0;
	mNumPrims=1;
	
	if(largeMesh->numIslands == 0) {
		return;
	}

	VecInt3 halfMax(-0xffff),halfMin(0xffff);
	for(uint16_t i=0;i<largeMesh->numIslands;i++) {
		PfxAABB16 aabb = largeMesh->aabbList[i];
		VecInt3 aabbMin((int32_t)XMin(aabb),(int32_t)YMin(aabb),(int32_t)ZMin(aabb));
		VecInt3 aabbMax((int32_t)XMax(aabb),(int32_t)YMax(aabb),(int32_t)ZMax(aabb));
		halfMax = maxPerElem(halfMax,aabbMax);
		halfMin = minPerElem(halfMin,aabbMin);
	}

	Vector3 vHalfMin = largeMesh->getWorldPosition(halfMin);
	Vector3 vHalfMax = largeMesh->getWorldPosition(halfMax);
	Vector3 vhalf = ( vHalfMax - vHalfMin ) * 0.5f;
	Vector3 vcenter = ( vHalfMin + vHalfMax ) * 0.5f;
	mCenter[0] = vcenter[0];
	mCenter[1] = vcenter[1];
	mCenter[2] = vcenter[2];
	mHalf[0] = vhalf[0];
	mHalf[1] = vhalf[1];
	mHalf[2] = vhalf[2];
	mCcdRadius = 0.0f;
}

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

#ifndef __COLLPRIM_H__
#define __COLLPRIM_H__


#include "Physics/Base/SimdFunc.h"

#include "Box.h"
#include "Capsule.h"
#include "Sphere.h"
#include "TriMesh.h"
#include "HeightField.h"

enum PrimType {
	SPHERE=0,		// 球
	BOX,			// ボックス
	CAPSULE,		// カプセル
	HEIGHTFIELD,	// ハイトフィールド
	CONVEXMESH,		// 凸ポリゴン
	TRIANGLEMESH,	// ポリゴン
	LARGEMESH,		// 大規模メッシュ
	PRIM_COUNT
};

class HeightField;

class CollPrim
{
friend class CollObject;

private:
	PrimType type;
	float relativeOrientation[4];
	float relativePosition[3];
	uint32_t contactFilterSelf;
	uint32_t contactFilterTarget;
	union {
		float    vfData[3];
		uint32_t viData[3];
	};

public:
	inline void reset()
	{
 		contactFilterSelf = contactFilterTarget = 0xffffffff;
 	}
	
	inline void setBox(Box box);
	inline void setCapsule(Capsule capsule);
	inline void setSphere(Sphere sphere);
	inline void setHeightField(const HeightField *heightfield);
	inline void setConvexMesh(const ConvexMesh *convexMesh);
	inline void setTriangleMesh(const TriMesh *triangleMesh);
	inline void setLargeMesh(const LargeTriMesh *largeMesh);
	inline void setObjectRelTransform(const Transform3 & xfrm);

	inline PrimType           getType() const;
	inline Box                getBox()const ;
	inline Capsule            getCapsule() const;
	inline Sphere             getSphere() const;
	inline Transform3         getObjectRelTransform() const;
	inline Quat               getObjectRelOrientation() const {return read_Quat(relativeOrientation);}
	inline Vector3            getObjectRelPosition() const {return read_Vector3(relativePosition);}

	inline void               setObjectRelOrientation(Quat rot) {return store_Quat(rot,relativeOrientation);}
	inline void 	          setObjectRelPosition(Vector3 pos) {return store_Vector3(pos,relativePosition);}
	
	inline void     setPrimDataFloat(int i,float v) {vfData[i]=v;}
	inline void     setPrimDataInteger(int i,uint32_t v) {viData[i]=v;}
	inline float    getPrimDataFloat(int i) const {return vfData[i];}
	inline uint32_t getPrimDataInteger(int i) const {return viData[i];}

	inline float    getBoundingRadius() const;

	void		setContactFilterSelf(uint32_t filter) {contactFilterSelf = filter;}
	uint32_t	getContactFilterSelf() const {return contactFilterSelf;}

	void		setContactFilterTarget(uint32_t filter) {contactFilterTarget = filter;}
	uint32_t	getContactFilterTarget() const {return contactFilterTarget;}

	// deform mesh
	inline void setPreLargeMesh(const LargeTriMesh *largeMesh) {viData[1]=(uint32_t)largeMesh;}
	inline LargeTriMesh *getPreLargeMesh() const {return (LargeTriMesh *)viData[1];}

	inline HeightField*       getHeightField() const;
	inline ConvexMesh*        getConvexMesh() const;
	inline LargeTriMesh*      getLargeMesh() const;
} __attribute__ ((aligned(16)));

inline
void
CollPrim::setBox(Box box)
{
	reset();
	vfData[0] = box.half[0];
	vfData[1] = box.half[1];
	vfData[2] = box.half[2];
	type = BOX;
}

inline
void
CollPrim::setCapsule(Capsule capsule)
{
	reset();
	vfData[0] = capsule.hLength;
	vfData[1] = capsule.radius;
	vfData[2] = 0.0f;
	type = CAPSULE;
}

inline
void
CollPrim::setSphere(Sphere sphere)
{
	reset();
	vfData[0] = sphere.radius;
	vfData[1] = 0.0f;
	vfData[2] = 0.0f;
	type = SPHERE;
}

inline
void
CollPrim::setHeightField(const HeightField *heightfield)
{
	reset();
	viData[0] = (uint32_t)heightfield;
	viData[1] = 0;
	viData[2] = 0;
	type = HEIGHTFIELD;
}

inline
void
CollPrim::setConvexMesh(const ConvexMesh *convexMesh)
{
	reset();
	viData[0] = (uint32_t)convexMesh;
	viData[1] = 0;
	viData[2] = 0;
	type = CONVEXMESH;
}

inline
void
CollPrim::setTriangleMesh(const TriMesh *triangleMesh)
{
	reset();
	viData[0] = (uint32_t)triangleMesh;
	viData[1] = 0;
	viData[2] = 0;
	type = TRIANGLEMESH;
}

inline
void
CollPrim::setLargeMesh(const LargeTriMesh *largeMesh)
{
	reset();
	viData[0] = (uint32_t)largeMesh;
	viData[1] = 0;
	viData[2] = 0;
	type = LARGEMESH;
}

inline
void
CollPrim::setObjectRelTransform(const Transform3 & xfrm)
{
	setObjectRelOrientation(Quat(xfrm.getUpper3x3()));
	setObjectRelPosition(xfrm.getTranslation());
}

inline
PrimType
CollPrim::getType() const
{
	return type;
}

inline
Box
CollPrim::getBox() const
{
	return Box(vfData[0],vfData[1],vfData[2]);
}

inline
Capsule
CollPrim::getCapsule() const
{
	return Capsule(vfData[0], vfData[1]);
}

inline
Sphere
CollPrim::getSphere() const
{
	return Sphere(vfData[0]);
}


inline
HeightField *
CollPrim::getHeightField() const
{
	return (HeightField*)viData[0];
}

inline
ConvexMesh *
CollPrim::getConvexMesh() const
{
	return (ConvexMesh*)viData[0];
}

inline
LargeTriMesh *
CollPrim::getLargeMesh() const
{
	return (LargeTriMesh*)viData[0];
}


inline
Transform3
CollPrim::getObjectRelTransform() const
{
	return Transform3(getObjectRelOrientation(),getObjectRelPosition());
}

inline float CollPrim::getBoundingRadius() const
{
	if ( getType() == BOX ) {
		return length(Vector3(vfData[0],vfData[1],vfData[2]));
	} else if ( getType() == CAPSULE ) {
		return vfData[0]+vfData[1];
	} else if ( getType() == SPHERE ) {
		return vfData[0];
//	} else if ( getType() == CONVEXMESH ) {
//		return length(getConvexMesh()->getAABB(getObjectRelTransform().getUpper3x3()));
	} else if ( getType() == LARGEMESH ) {
		return 1000.0f; // TODO
	}
	return 0.0f;
}

#endif /* __COLLPRIM_H__ */


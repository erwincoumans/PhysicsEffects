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

#ifndef __PFX_COMMON_IMPORT_H__
#define __PFX_COMMON_IMPORT_H__

#include "Plugins/PfxCommonFormat/PfxCommonFileIO.h"
#include "Plugins/PfxCommonFormat/PfxCommonFormat.h"

///////////////////////////////////////////////////////////////////////////////
// PfxCommonImport

class PfxCommonImport : public PfxCommonFileReader , public PfxCommonFormat
{
private:
	void readRigidBodies();
	
	void readRigBodies();
	void readRigStates();
	void readRigJoints();
	void readRigConvexMeshes();
	void readRigLargeMeshes();
	void readRigNonContactPairs();
	
	PfxBool readRigWorld(PfxRigWorldInfo &world);
	PfxBool readRigBody(PfxRigBodyInfo &body);
	PfxBool readRigShape(PfxRigShapeInfo &shape);
	PfxBool readRigState(PfxRigStateInfo &state);
	PfxBool readRigJoint(PfxRigJointInfo &joint);
	PfxBool readRigConvexMesh(PfxRigConvexMeshInfo &convexmesh);
	PfxBool readRigLargeMesh(PfxRigLargeMeshInfo &largemesh);
	PfxBool readPair(PfxRigNonContactPairInfo &pair);
	
	void readVerts(vector<PfxFloat> &verts);
	void readIndices(vector<PfxUInt16> &indices);
	
public:
	
	PfxBool doImport(const char *filename);
};

#endif /* __PFX_COMMON_IMPORT_H__ */

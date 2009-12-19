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

#ifndef __PFX_SNAPOSHOT_READER_H__
#define __PFX_SNAPOSHOT_READER_H__

#include "Physics/RigidBody/RigidBodies.h"
#include "Physics/Base/PfxFileIO.h"

#include <vector>
#include <map>
using namespace std;

///////////////////////////////////////////////////////////////////////////////
// PfxSnapshotReader

class PfxSnapshotReader : public PfxFileReader
{
private:
	RigidBodies *mRigidBodies;

	map<uint32_t,ConvexMesh>	*mMeshArray;
	map<uint32_t,LargeTriMesh>	*mLargeMeshArray;

	void readTrbDynBodyArray();
	void readTrbStateArray();
	void readJointArray();
	void readConvexMeshArray();
	void readLargeMeshArray();
	void readNonContactPair();
	
	bool readWorldProperty(WorldProperty &worldProperty);
	bool readTrbDynBody();
	bool readPrim(CollObject &coll);
	bool readTrbState();
	bool readJoint();

	uint32_t readTriMesh(TriMesh &mesh);
	uint32_t readConvexMesh(ConvexMesh &mesh);
	uint32_t readLargeMesh(LargeTriMesh &lmesh);
	
	void readVerts(vector<float> &verts);
	void readIndices(vector<uint16_t> &indices);
	
public:
	bool getWorldProperty(const char *filename,
		WorldProperty &worldProperty);
	
	bool doImport(const char *filename,
		RigidBodies *rigidbodies,
		map<uint32_t,ConvexMesh>	*meshArray,
		map<uint32_t,LargeTriMesh>	*largeMeshArray
		);
};

#endif /* __PFX_SNAPOSHOT_READER_H__ */

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

#ifndef __RENDER_UTIL_H__
#define __RENDER_UTIL_H__

#ifdef WIN32
	#include <windows.h>
	#include <gl/gl.h>
	#include <gl/glu.h>
#endif

#ifdef __PPU__
	#include <PSGL/psgl.h>
	#include <PSGL/psglu.h>
#endif

class RenderUtil {
public:
	struct Mesh {
		float		*vtx;
		float		*nml;
		unsigned int	numVerts;
		unsigned short	*idx;
		unsigned int	numIndices;
	};
	
private:
	Mesh mBox,mSphere,mCylinder;
	Mesh mTmpBox,mTmpSphere,mTmpCylinder;
	
	void createBox(Mesh &mesh);
	void createSphere(Mesh &mesh);
	void createCylinder(Mesh &mesh);
	
public:
	RenderUtil();
	~RenderUtil();

	void calcNormal(Mesh &mesh);

	void drawLine(float *p1,float *p2);
	void drawStar(float *p,float extend);
	void drawAxis(float len);
	void drawFrame(float *center,float *halfExtend);

	void drawBoxSilhouette(float hx,float hy,float hz);
	void drawSphereSilhouette(float r);
	void drawCapsuleSilhouette(float hl,float r);
	void drawMeshSilhouette(Mesh &mesh) {drawMeshSilhouette(mesh.vtx,mesh.nml,mesh.idx,mesh.numVerts,mesh.numIndices);}
	void drawMeshSilhouette(float *vtx,float *nml,unsigned short *idx,int numVerts,int numIndices);

	void drawBox(float hx,float hy,float hz);
	void drawSphere(float r);
	void drawCapsule(float hl,float r);
	void drawMesh(Mesh &mesh) {drawMesh(mesh.vtx,mesh.nml,mesh.idx,mesh.numVerts,mesh.numIndices);}
	void drawMesh(float *vtx,float *nml,unsigned short *idx,int numVerts,int numIndices);
	void drawConvexHull(float *vtx,int numVerts);
};

#endif /* __RENDER_UTIL_H__ */

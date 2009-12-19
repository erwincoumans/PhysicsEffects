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

#include <math.h>
#include <float.h>

#include <vectormath_aos.h>
using namespace Vectormath::Aos;

#include "RenderUtil.h"
#include "Physics/Base/PhysicsCommon.h"
#include "Util/PfxConvexCreater.h"

#define AddVector3(array,x,y,z,count) {array[count*3] = (float)x;array[count*3+1] = (float)y;array[count*3+2] = (float)z;count++;}
#define AddRotatedVector3(array,x,y,z,q,count) {Vector3 pos = rotate(q,Vector3(x,y,z));array[count*3] = (float)pos[0];array[count*3+1] = (float)pos[1];array[count*3+2] = (float)pos[2];count++;}

///////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor

RenderUtil::RenderUtil()
{
	createBox(mBox);
	createSphere(mSphere);
	createCylinder(mCylinder);
	createBox(mTmpBox);
	createSphere(mTmpSphere);
	createCylinder(mTmpCylinder);
}

RenderUtil::~RenderUtil()
{
	delete mBox.vtx;
	delete mBox.nml;
	delete mBox.idx;
	delete mSphere.vtx;
	delete mSphere.nml;
	delete mSphere.idx;
	delete mCylinder.vtx;
	delete mCylinder.nml;
	delete mCylinder.idx;
	delete mTmpBox.vtx;
	delete mTmpBox.nml;
	delete mTmpBox.idx;
	delete mTmpSphere.vtx;
	delete mTmpSphere.nml;
	delete mTmpSphere.idx;
	delete mTmpCylinder.vtx;
	delete mTmpCylinder.nml;
	delete mTmpCylinder.idx;
}

///////////////////////////////////////////////////////////////////////////////
// Create Mesh

void RenderUtil::createBox(Mesh &mesh)
{
	int vtxcount = 36;
	mesh.vtx = new float[vtxcount*3];
	mesh.nml = new float[vtxcount*3];
	mesh.idx = new unsigned short[vtxcount];
	int numvtx,numnml;
	numvtx=numnml=0;

	// -Z
	AddVector3(mesh.vtx,-0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx,-0.5f, 0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f,-0.5f,numvtx);
	AddVector3(mesh.nml, 0.0f, 0.0f,-1.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 0.0f,-1.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 0.0f,-1.0f,numnml);
	
	AddVector3(mesh.vtx,-0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.nml, 0.0f, 0.0f,-1.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 0.0f,-1.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 0.0f,-1.0f,numnml);

	// -Y
	AddVector3(mesh.vtx,-0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f,-0.5f, 0.5f,numvtx);
	AddVector3(mesh.nml, 0.0f,-1.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 0.0f,-1.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 0.0f,-1.0f, 0.0f,numnml);
	
	AddVector3(mesh.vtx,-0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f,-0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx,-0.5f,-0.5f, 0.5f,numvtx);
	AddVector3(mesh.nml, 0.0f,-1.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 0.0f,-1.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 0.0f,-1.0f, 0.0f,numnml);
	
	// -X
	AddVector3(mesh.vtx,-0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx,-0.5f,-0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx,-0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.nml,-1.0f, 0.0f, 0.0f,numnml);
	AddVector3(mesh.nml,-1.0f, 0.0f, 0.0f,numnml);
	AddVector3(mesh.nml,-1.0f, 0.0f, 0.0f,numnml);
	
	AddVector3(mesh.vtx,-0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx,-0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx,-0.5f, 0.5f,-0.5f,numvtx);
	AddVector3(mesh.nml,-1.0f, 0.0f, 0.0f,numnml);
	AddVector3(mesh.nml,-1.0f, 0.0f, 0.0f,numnml);
	AddVector3(mesh.nml,-1.0f, 0.0f, 0.0f,numnml);
	
	// +Z
	AddVector3(mesh.vtx,-0.5f,-0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f,-0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.nml, 0.0f, 0.0f, 1.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 0.0f, 1.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 0.0f, 1.0f,numnml);
	
	AddVector3(mesh.vtx,-0.5f,-0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx,-0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.nml, 0.0f, 0.0f, 1.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 0.0f, 1.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 0.0f, 1.0f,numnml);

	// +Y
	AddVector3(mesh.vtx,-0.5f, 0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx,-0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.nml, 0.0f, 1.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 1.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 1.0f, 0.0f,numnml);
	
	AddVector3(mesh.vtx,-0.5f, 0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f,-0.5f,numvtx);
	AddVector3(mesh.nml, 0.0f, 1.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 1.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 0.0f, 1.0f, 0.0f,numnml);
	
	// +X
	AddVector3(mesh.vtx, 0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.nml, 1.0f, 0.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 1.0f, 0.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 1.0f, 0.0f, 0.0f,numnml);
	
	AddVector3(mesh.vtx, 0.5f,-0.5f,-0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f, 0.5f, 0.5f,numvtx);
	AddVector3(mesh.vtx, 0.5f,-0.5f, 0.5f,numvtx);
	AddVector3(mesh.nml, 1.0f, 0.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 1.0f, 0.0f, 0.0f,numnml);
	AddVector3(mesh.nml, 1.0f, 0.0f, 0.0f,numnml);

	int numidx = 0;

	for(int i=0;i<vtxcount;i++) {
		mesh.idx[numidx++] = (unsigned short)i;
	}

	mesh.numVerts = numvtx;
	mesh.numIndices = numidx;
}

void RenderUtil::createSphere(Mesh &mesh)
{
	int div = 12;
	int vtxcount = 2+(div+1)*(div-2)/2;
	mesh.vtx = new float[vtxcount*3];
	mesh.nml = new float[vtxcount*3];

	int i,j;
	int D=div;
	int D2=D/2;
	int D4=D/4;
	float dR = 2*3.1415926535f/(float)D;
	float dU = 1/(float)D;
	float dV = 2/(float)D;
	
	unsigned int numvtx,numnml;
	numvtx=numnml=0;
	
	// 頂点作成
	for(i=-D4;i<=D4;i++) {
		if(i == -D4) {
			AddVector3(mesh.vtx,0.0f,-1.0f,0.0f,numvtx);
			AddVector3(mesh.nml,0.0f,-1.0f,0.0f,numnml);
		}
		else if(i == D4) {
			AddVector3(mesh.vtx,0.0f,1.0f,0.0f,numvtx);
			AddVector3(mesh.nml,0.0f,1.0f,0.0f,numnml);
		}
		else {
			float R = cosf(i*dR);
			for(j=0;j<=D;j++) {
				AddVector3(mesh.vtx,R*cosf(j*dR),sinf(i*dR),R*sinf(j*dR),numvtx);
				AddVector3(mesh.nml,R*cosf(j*dR),sinf(i*dR),R*sinf(j*dR),numnml);
			}
		}
	}
	
	mesh.numVerts = numvtx;
	
	// インデックス作成
	int dcnt = D+1;
	int idxcount = div*(div-2)*3;
	int numidx = 0;

	mesh.idx = new unsigned short[idxcount];

	for(i=0;i<D2;i++) {
		if(i == 0) {
			for(j=0;j<D;j++) {
				mesh.idx[numidx++] = 0;
				mesh.idx[numidx++] = -div+(i+1)*dcnt+j;
				mesh.idx[numidx++] = -div+(i+1)*dcnt+(j+1)%dcnt;
			}
		}
		else if(i == D2-1) {
			for(j=0;j<D;j++) {
				mesh.idx[numidx++] = -div+i*dcnt+(j+1)%dcnt;
				mesh.idx[numidx++] = -div+i*dcnt+j;
				mesh.idx[numidx++] = vtxcount-1;
			}
		}
		else {
			for(j=0;j<D;j++) {
				mesh.idx[numidx++] = -div+i*dcnt+j;
				mesh.idx[numidx++] = -div+(i+1)*dcnt+j;
				mesh.idx[numidx++] = -div+(i+1)*dcnt+(j+1)%dcnt;

				mesh.idx[numidx++] = -div+(i+1)*dcnt+(j+1)%dcnt;
				mesh.idx[numidx++] = -div+i*dcnt+(j+1)%dcnt;
				mesh.idx[numidx++] = -div+i*dcnt+j;
			}
		}
	}

	mesh.numIndices = numidx;
}

void RenderUtil::createCylinder(Mesh &mesh)
{
	Quat rot = Quat::rotationZ(-3.1415926535f*0.5f);
	
	int div = 12;
	int vtxcount = (div+1)*2;
	mesh.vtx = new float[vtxcount*3];
	mesh.nml = new float[vtxcount*3];

	float radius = 1.0f;
	float hlen = 0.5f;

	int j;
	int D=div;
	float dR = 2*3.1415926535f/(float)D;
	float dU = 1/(float)D;

	unsigned int numvtx,numnml;
	numvtx=numnml=0;
	
	// 頂点作成
	for(j=0;j<=D;j++) {
		AddRotatedVector3(mesh.vtx,radius*cosf(j*dR),-hlen,radius*sinf(j*dR),rot,numvtx);
		AddRotatedVector3(mesh.nml,cosf(j*dR),0.0f,sinf(j*dR),rot,numnml);
	}
	for(j=0;j<=D;j++) {
		AddRotatedVector3(mesh.vtx,radius*cosf(j*dR), hlen,radius*sinf(j*dR),rot,numvtx);
		AddRotatedVector3(mesh.nml,cosf(j*dR),0.0f,sinf(j*dR),rot,numnml);
	}
	
	mesh.numVerts = numvtx;
	
	// インデックス作成
	int dcnt = D+1;
	int idxcount = div*2*3;
	int numidx = 0;
	mesh.idx = new unsigned short[idxcount];
	for(j=0;j<D;j++) {
		mesh.idx[numidx++] = j;
		mesh.idx[numidx++] = dcnt+j;
		mesh.idx[numidx++] = dcnt+(j+1)%dcnt;

		mesh.idx[numidx++] = dcnt+(j+1)%dcnt;
		mesh.idx[numidx++] = (j+1)%dcnt;
		mesh.idx[numidx++] = j;
	}
	
	mesh.numIndices = numidx;
}

///////////////////////////////////////////////////////////////////////////////
// Draw Method

void RenderUtil::drawLine(float *p1,float *p2)
{
	GLfloat vtx[6];
	
	vtx[0] = p1[0];
	vtx[1] = p1[1];
	vtx[2] = p1[2];
	vtx[3] = p2[0];
	vtx[4] = p2[1];
	vtx[5] = p2[2];

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,0,&vtx[0]);
	glDrawArrays(GL_LINES,0,2);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void RenderUtil::drawStar(float *p,float extend)
{
	GLfloat vtx[18];

	GLfloat x = p[0];
	GLfloat y = p[1];
	GLfloat z = p[2];

	vtx[0]  = x+extend;	vtx[1]  = y;		vtx[2]  = z;
	vtx[3]  = x-extend;	vtx[4]  = y;		vtx[5]  = z;
	vtx[6]  = x;		vtx[7]  = y+extend;	vtx[8]  = z;
	vtx[9]  = x;		vtx[10] = y-extend;	vtx[11] = z;
	vtx[12] = x;		vtx[13] = y;		vtx[14] = z+extend;
	vtx[15] = x;		vtx[16] = y;		vtx[17] = z-extend;

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,0,&vtx[0]);
	glDrawArrays(GL_LINES,0,2);
	glVertexPointer(3,GL_FLOAT,0,&vtx[6]);
	glDrawArrays(GL_LINES,0,2);
	glVertexPointer(3,GL_FLOAT,0,&vtx[12]);
	glDrawArrays(GL_LINES,0,2);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void RenderUtil::drawAxis(float len)
{
	glBegin(GL_LINES);

	// X Axis
	glColor4f(1,0,0,1);
	glVertex3f( 0.0f, 0.0f, 0.0f );
	glVertex3f( len, 0.0f, 0.0f );
	glVertex3f( len, 0.0f, 0.0f );
	glVertex3f(0.8f*len, 0.2f*len, 0.0f );
	glVertex3f( len, 0.0f, 0.0f );
	glVertex3f(0.8f*len,-0.2f*len, 0.0f );

	// Y Axis
	glColor4f(0,1,0,1);
	glVertex3f( 0.0f, 0.0f, 0.0f );
	glVertex3f( 0.0f, len, 0.0f );
	glVertex3f( 0.0f, len, 0.0f );
	glVertex3f( 0.0f, 0.8f*len, 0.2f*len );
	glVertex3f( 0.0f, len, 0.0f );
	glVertex3f( 0.0f, 0.8f*len,-0.2f*len );

	// Z Axis
	glColor4f(0,0,1,1);
	glVertex3f( 0.0f, 0.0f, 0.0f );
	glVertex3f( 0.0f, 0.0f, len );
	glVertex3f( 0.0f, 0.0f, len );
	glVertex3f( 0.0f, 0.2f*len, 0.8f*len );
	glVertex3f( 0.0f, 0.0f, len );
	glVertex3f( 0.0f,-0.2f*len, 0.8f*len );

	glEnd();
}

void RenderUtil::drawFrame(float *center,float *halfExtend)
{
	GLfloat vtx[48];

	vtx[0] = center[0] + halfExtend[0];
	vtx[1] = center[1] - halfExtend[1];
	vtx[2] = center[2] - halfExtend[2];
	vtx[3] = center[0] + halfExtend[0];
	vtx[4] = center[1] - halfExtend[1];
	vtx[5] = center[2] + halfExtend[2];
	vtx[6] = center[0] - halfExtend[0];
	vtx[7] = center[1] - halfExtend[1];
	vtx[8] = center[2] + halfExtend[2];
	vtx[9] = center[0] - halfExtend[0];
	vtx[10] = center[1] - halfExtend[1];
	vtx[11] = center[2] - halfExtend[2];

	vtx[12] = center[0] + halfExtend[0];
	vtx[13] = center[1] + halfExtend[1];
	vtx[14] = center[2] - halfExtend[2];
	vtx[15] = center[0] + halfExtend[0];
	vtx[16] = center[1] + halfExtend[1];
	vtx[17] = center[2] + halfExtend[2];
	vtx[18] = center[0] - halfExtend[0];
	vtx[19] = center[1] + halfExtend[1];
	vtx[20] = center[2] + halfExtend[2];
	vtx[21] = center[0] - halfExtend[0];
	vtx[22] = center[1] + halfExtend[1];
	vtx[23] = center[2] - halfExtend[2];

	vtx[24] = center[0] + halfExtend[0];
	vtx[25] = center[1] - halfExtend[1];
	vtx[26] = center[2] - halfExtend[2];
	vtx[27] = center[0] + halfExtend[0];
	vtx[28] = center[1] + halfExtend[1];
	vtx[29] = center[2] - halfExtend[2];

	vtx[30] = center[0] + halfExtend[0];
	vtx[31] = center[1] - halfExtend[1];
	vtx[32] = center[2] + halfExtend[2];
	vtx[33] = center[0] + halfExtend[0];
	vtx[34] = center[1] + halfExtend[1];
	vtx[35] = center[2] + halfExtend[2];

	vtx[36] = center[0] - halfExtend[0];
	vtx[37] = center[1] - halfExtend[1];
	vtx[38] = center[2] + halfExtend[2];
	vtx[39] = center[0] - halfExtend[0];
	vtx[40] = center[1] + halfExtend[1];
	vtx[41] = center[2] + halfExtend[2];

	vtx[42] = center[0] - halfExtend[0];
	vtx[43] = center[1] - halfExtend[1];
	vtx[44] = center[2] - halfExtend[2];
	vtx[45] = center[0] - halfExtend[0];
	vtx[46] = center[1] + halfExtend[1];
	vtx[47] = center[2] - halfExtend[2];

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,0,&vtx[0]);
	glDrawArrays(GL_LINE_LOOP,0,4);
	glVertexPointer(3,GL_FLOAT,0,&vtx[12]);
	glDrawArrays(GL_LINE_LOOP,0,4);
	glVertexPointer(3,GL_FLOAT,0,&vtx[24]);
	glDrawArrays(GL_LINES,0,8);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void RenderUtil::drawBoxSilhouette(float hx,float hy,float hz)
{
	for(int i=0;i<mBox.numVerts;i++) {
		mTmpBox.vtx[i*3  ] = 2.0f*hx*mBox.vtx[i*3  ];
		mTmpBox.vtx[i*3+1] = 2.0f*hy*mBox.vtx[i*3+1];
		mTmpBox.vtx[i*3+2] = 2.0f*hz*mBox.vtx[i*3+2];
	}

	glPushMatrix();

	glEnableClientState(GL_VERTEX_ARRAY);

	glVertexPointer(3,GL_FLOAT,0,mTmpBox.vtx);
	glDrawElements(GL_TRIANGLES,mBox.numIndices,GL_UNSIGNED_SHORT,mBox.idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

void RenderUtil::drawSphereSilhouette(float r)
{
	for(int i=0;i<mSphere.numVerts;i++) {
		mTmpSphere.vtx[i*3  ] = r*mSphere.vtx[i*3  ];
		mTmpSphere.vtx[i*3+1] = r*mSphere.vtx[i*3+1];
		mTmpSphere.vtx[i*3+2] = r*mSphere.vtx[i*3+2];
	}

	glPushMatrix();

	glEnableClientState(GL_VERTEX_ARRAY);

	glVertexPointer(3,GL_FLOAT,0,mTmpSphere.vtx);
	glDrawElements(GL_TRIANGLES,mSphere.numIndices,GL_UNSIGNED_SHORT,mSphere.idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

void RenderUtil::drawCapsuleSilhouette(float hl,float r)
{
	for(int i=0;i<mSphere.numVerts;i++) {
		mTmpSphere.vtx[i*3  ] = r*mSphere.vtx[i*3  ];
		mTmpSphere.vtx[i*3+1] = r*mSphere.vtx[i*3+1];
		mTmpSphere.vtx[i*3+2] = r*mSphere.vtx[i*3+2];
	}

	for(int i=0;i<mCylinder.numVerts;i++) {
		mTmpCylinder.vtx[i*3  ] = 2.0f*hl*mCylinder.vtx[i*3  ];
		mTmpCylinder.vtx[i*3+1] = r*mCylinder.vtx[i*3+1];
		mTmpCylinder.vtx[i*3+2] = r*mCylinder.vtx[i*3+2];
	}

	glEnableClientState(GL_VERTEX_ARRAY);

	// Sphere x2
	glPushMatrix();
	glTranslatef(-hl,0.0f,0.0f);
	glVertexPointer(3,GL_FLOAT,0,mTmpSphere.vtx);
	glDrawElements(GL_TRIANGLES,mSphere.numIndices,GL_UNSIGNED_SHORT,mSphere.idx);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(hl,0.0f,0.0f);
	glVertexPointer(3,GL_FLOAT,0,mTmpSphere.vtx);
	glDrawElements(GL_TRIANGLES,mSphere.numIndices,GL_UNSIGNED_SHORT,mSphere.idx);
	glPopMatrix();
	
	// Cylinder x1
	glPushMatrix();

	glVertexPointer(3,GL_FLOAT,0,mTmpCylinder.vtx);
	glDrawElements(GL_TRIANGLES,mCylinder.numIndices,GL_UNSIGNED_SHORT,mCylinder.idx);

	glPopMatrix();

	glDisableClientState(GL_VERTEX_ARRAY);
}

void RenderUtil::drawMeshSilhouette(float *vtx,float *nml,unsigned short *idx,int numVerts,int numIndices)
{
	glPushMatrix();

	glEnableClientState(GL_VERTEX_ARRAY);

	glVertexPointer(3,GL_FLOAT,0,vtx);
	glDrawElements(GL_TRIANGLES,numIndices,GL_UNSIGNED_SHORT,idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

void RenderUtil::drawBox(float hx,float hy,float hz)
{
	for(int i=0;i<mBox.numVerts;i++) {
		mTmpBox.vtx[i*3  ] = 2.0f*hx*mBox.vtx[i*3  ];
		mTmpBox.vtx[i*3+1] = 2.0f*hy*mBox.vtx[i*3+1];
		mTmpBox.vtx[i*3+2] = 2.0f*hz*mBox.vtx[i*3+2];
	}

	glPushMatrix();

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3,GL_FLOAT,0,mTmpBox.vtx);
	glNormalPointer(GL_FLOAT,0,mBox.nml);
	glDrawElements(GL_TRIANGLES,mBox.numIndices,GL_UNSIGNED_SHORT,mBox.idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glPopMatrix();
}

void RenderUtil::drawSphere(float r)
{
	for(int i=0;i<mSphere.numVerts;i++) {
		mTmpSphere.vtx[i*3  ] = r*mSphere.vtx[i*3  ];
		mTmpSphere.vtx[i*3+1] = r*mSphere.vtx[i*3+1];
		mTmpSphere.vtx[i*3+2] = r*mSphere.vtx[i*3+2];
	}

	glPushMatrix();

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3,GL_FLOAT,0,mTmpSphere.vtx);
	glNormalPointer(GL_FLOAT,0,mSphere.nml);
	glDrawElements(GL_TRIANGLES,mSphere.numIndices,GL_UNSIGNED_SHORT,mSphere.idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glPopMatrix();
}

void RenderUtil::drawCapsule(float hl,float r)
{
	for(int i=0;i<mSphere.numVerts;i++) {
		mTmpSphere.vtx[i*3  ] = r*mSphere.vtx[i*3  ];
		mTmpSphere.vtx[i*3+1] = r*mSphere.vtx[i*3+1];
		mTmpSphere.vtx[i*3+2] = r*mSphere.vtx[i*3+2];
	}

	for(int i=0;i<mCylinder.numVerts;i++) {
		mTmpCylinder.vtx[i*3  ] = 2.0f*hl*mCylinder.vtx[i*3  ];
		mTmpCylinder.vtx[i*3+1] = r*mCylinder.vtx[i*3+1];
		mTmpCylinder.vtx[i*3+2] = r*mCylinder.vtx[i*3+2];
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	// Sphere x2
	glPushMatrix();
	glTranslatef(-hl,0.0f,0.0f);
	glVertexPointer(3,GL_FLOAT,0,mTmpSphere.vtx);
	glNormalPointer(GL_FLOAT,0,mSphere.nml);
	glDrawElements(GL_TRIANGLES,mSphere.numIndices,GL_UNSIGNED_SHORT,mSphere.idx);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(hl,0.0f,0.0f);
	glVertexPointer(3,GL_FLOAT,0,mTmpSphere.vtx);
	glNormalPointer(GL_FLOAT,0,mSphere.nml);
	glDrawElements(GL_TRIANGLES,mSphere.numIndices,GL_UNSIGNED_SHORT,mSphere.idx);
	glPopMatrix();
	
	// Cylinder x1
	glPushMatrix();

	//glScalef(2.0f*hl,r,r);

	glVertexPointer(3,GL_FLOAT,0,mTmpCylinder.vtx);
	glNormalPointer(GL_FLOAT,0,mCylinder.nml);
	glDrawElements(GL_TRIANGLES,mCylinder.numIndices,GL_UNSIGNED_SHORT,mCylinder.idx);

	glPopMatrix();

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

void RenderUtil::drawMesh(float *vtx,float *nml,unsigned short *idx,int numVerts,int numIndices)
{
	glPushMatrix();

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3,GL_FLOAT,0,vtx);
	glNormalPointer(GL_FLOAT,0,nml);
	glDrawElements(GL_TRIANGLES,numIndices,GL_UNSIGNED_SHORT,idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glPopMatrix();
}

void RenderUtil::calcNormal(Mesh &mesh)
{
	float *nmlcnt = new float[mesh.numVerts];
	memset(nmlcnt,0,sizeof(float)*mesh.numVerts);
	memset(mesh.nml,0,sizeof(float)*mesh.numVerts*3);

	for(unsigned int j=0;j<mesh.numIndices/3;j++) {
		unsigned short p0 = mesh.idx[j*3+0];
		unsigned short p1 = mesh.idx[j*3+1];
		unsigned short p2 = mesh.idx[j*3+2];
		Vector3 v0(mesh.vtx[p0*3],mesh.vtx[p0*3+1],mesh.vtx[p0*3+2]);
		Vector3 v1(mesh.vtx[p1*3],mesh.vtx[p1*3+1],mesh.vtx[p1*3+2]);
		Vector3 v2(mesh.vtx[p2*3],mesh.vtx[p2*3+1],mesh.vtx[p2*3+2]);
		Vector3 facetnml = normalize(cross(v2-v1,v0-v1));
		mesh.nml[p0*3  ] += facetnml[0];
		mesh.nml[p0*3+1] += facetnml[1];
		mesh.nml[p0*3+2] += facetnml[2];
		mesh.nml[p1*3  ] += facetnml[0];
		mesh.nml[p1*3+1] += facetnml[1];
		mesh.nml[p1*3+2] += facetnml[2];
		mesh.nml[p2*3  ] += facetnml[0];
		mesh.nml[p2*3+1] += facetnml[1];
		mesh.nml[p2*3+2] += facetnml[2];
		nmlcnt[p0] += 1.0f;
		nmlcnt[p1] += 1.0f;
		nmlcnt[p2] += 1.0f;
	}

	for(unsigned int j=0;j<mesh.numVerts;j++) {
		float x = mesh.nml[j*3  ]/nmlcnt[j];
		float y = mesh.nml[j*3+1]/nmlcnt[j];
		float z = mesh.nml[j*3+2]/nmlcnt[j];
		float l = sqrtf(x*x+y*y+z*z);
		mesh.nml[j*3  ] = x/l;
		mesh.nml[j*3+1] = y/l;
		mesh.nml[j*3+2] = z/l;
	}

	delete nmlcnt;
}

void RenderUtil::drawConvexHull(float *vtx,int numVerts)
{
	vector<CHPolygon> polys;

	//createConvexPolygons(CH_USE_6DOP,vtx,numVerts,polys);
	//createConvexPolygons(CH_USE_14DOP,vtx,numVerts,polys);
	createConvexPolygons(CH_USE_26DOP,vtx,numVerts,polys);

	glDisable(GL_LIGHTING);
	for(int i=0;i<polys.size();i++) {
		glColor4f(1,0,0,1);
		glBegin(GL_LINE_LOOP);
		for(int j=0;j<polys[i].verts.size();j++) {
			Vector3 v = 0.1f * polys[i].plane.N + polys[i].verts[j];
			glVertex3fv((float*)&v);
		}
		glEnd();

		glColor4f(0,0,1,1);
		glBegin(GL_LINES);
		for(int j=1;j<polys[i].verts.size()-1;j++) {
			Vector3 v1 = 0.1f * polys[i].plane.N + polys[i].verts[0];
			Vector3 v2 = 0.1f * polys[i].plane.N + polys[i].verts[j];
			glVertex3fv((float*)&v1);
			glVertex3fv((float*)&v2);
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);
}

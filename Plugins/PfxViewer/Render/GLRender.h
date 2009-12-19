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
#ifndef __GLRENDER_H__
#define __GLRENDER_H__

#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>
#include "GLFont.h"

// レンダラーモード
enum RenderMode {
	RenderModeNormal,
	RenderModeDraft
};

class GLRender
{
private:
	HWND mHWnd;
	HDC mHDc;
    HGLRC mHRC;
	float mWidth;
	float mHeight;
	RenderMode mRenderMode;
	bool mActive;

	HGLRC createGLRC(HDC _hdc,bool offscreen);
	int selectHits(GLuint hits,GLuint *buf,float *depth);

public:
	void setHWND(HWND hwnd);
	HWND getHWND() {return mHWnd;}
	
	bool getActive() {return mActive;}

	void setWidth(float w) {mWidth = w;}
	float getWidth() {return mWidth;}
	
	void setHeight(float h) {mHeight = h;}
	float getHeight() {return mHeight;}

	void setVSync(bool sw);
	
public:
	GLRender();
	virtual ~GLRender() {}

	bool initializeContext();
	void finalizeContext();
	
	void drawSceneBegin();
	void drawSceneEnd();
};

#endif

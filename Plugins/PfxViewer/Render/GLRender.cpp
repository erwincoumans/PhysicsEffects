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

#include "GLRender.h"

GLRender::GLRender()
{
	mHDc = NULL;
	mHRC = NULL;
	mWidth = mHeight = 0;
	mActive = false;
	mRenderMode = RenderModeNormal;
}

bool GLRender::initializeContext()
{
	// デバイスコンテキストの取得
	mHRC = createGLRC(mHDc,false);
	if (!mHRC) return false;

	mActive = true;

	wglMakeCurrent(mHDc, mHRC);
	glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glViewport(0, 0, mWidth,mHeight);

	glFontInitialize(mHDc);

	return true;
}

void GLRender::finalizeContext()
{
	if(!mActive) return;

	glFontRelease();

    wglMakeCurrent(0,0);
    wglDeleteContext(mHRC);

	ReleaseDC(mHWnd,mHDc);
}

void GLRender::setHWND(HWND hwnd)
{
	mHWnd = hwnd;
	mHDc = GetDC(mHWnd);
}

//OpenGLレンダリングコンテキストの作成
HGLRC GLRender::createGLRC(HDC _hdc,bool offscreen)
{
	int pixelformat;
	DWORD dwFlags;
	HGLRC _glrc;

	if(offscreen)
		dwFlags = PFD_DRAW_TO_BITMAP | PFD_SUPPORT_OPENGL | PFD_SUPPORT_GDI;
	else
		dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
		
    static PIXELFORMATDESCRIPTOR pfd = {
		sizeof(PIXELFORMATDESCRIPTOR),		//この構造体のサイズ
		1,									//OpenGLバージョン
		dwFlags,
		PFD_TYPE_RGBA,						//RGBAカラー
		32,									//色数
		0, 0,								//RGBAのビットとシフト設定		
		0, 0,								//G
		0, 0,								//B
		0, 0,								//A
		0,									//アキュムレーションバッファ
		0, 0, 0, 0, 						//RGBAアキュムレーションバッファ
		32,									//Zバッファ	
		0,									//ステンシルバッファ
		0,									//使用しない
		PFD_MAIN_PLANE,						//レイヤータイプ
		0,									//予約
		0, 0, 0								//レイヤーマスクの設定・未使用
    };

	//ピクセルフォーマットの指定
    if ( (pixelformat = ChoosePixelFormat(_hdc, &pfd)) == 0 ){
		OutputDebugString("ChoosePixelFormat Failed....");
        return NULL;
    }
	//ピクセルフォーマットの設定
    if (SetPixelFormat(_hdc, pixelformat, &pfd) == FALSE){
		OutputDebugString("SetPixelFormat Failed....");
        return NULL;
    }
	//OpenGLレンダリングコンテキストの作成
	if (!(_glrc=wglCreateContext(_hdc))){
		OutputDebugString("Creating HGLRC Failed....");
		return NULL;
	}

	return _glrc;
}

void GLRender::drawSceneBegin()
{
	if(!mActive) return;

	wglMakeCurrent(mHDc, mHRC);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glViewport(0, 0, mWidth,mHeight);
}

void GLRender::drawSceneEnd()
{
    SwapBuffers(wglGetCurrentDC());
}

void GLRender::setVSync(bool sw)
{
	BOOL (WINAPI *wglSwapIntervalEXT)(int) = NULL;

	if(strstr((char*)glGetString( GL_EXTENSIONS ),"WGL_EXT_swap_control")== 0) {
		// Can't initialize WGL_EXT_swap_control extension
	}
	else {
		wglSwapIntervalEXT = (BOOL (WINAPI*)(int))wglGetProcAddress("wglSwapIntervalEXT");
		if(wglSwapIntervalEXT) wglSwapIntervalEXT(sw?1:0);
	}
}

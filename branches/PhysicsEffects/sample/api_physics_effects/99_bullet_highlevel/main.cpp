﻿/* SCE CONFIDENTIAL
 * PSP2 Programmer Tool Runtime Library Release 00.930.000
 *                Copyright (C) 2010 Sony Computer Entertainment Inc.
 *                                                All Rights Reserved.
 */

#include "btBulletDynamicsCommon.h"
#include "../common/common.h"
#include "../common/ctrl_func.h"
#include "../common/render_func.h"
#include "../common/perf_func.h"
#include "physics_func.h"
#include "barrel.h"
#include "landscape.h"

#ifdef WIN32
	#include <gl/gl.h>
	#include <gl/glu.h>
#endif

#define	SAMPLE_NAME "api_physics_effects/1_simple"

static bool s_isRunning = true;

int sceneId = 2;
bool simulating = false;

int landscapeMeshId;
int convexMeshId;

/*
	kPfxShapeSphere = 0,
	kPfxShapeBox,		
	kPfxShapeCapsule,	
	kPfxShapeCylinder,	
	kPfxShapeConvexMesh,
	kPfxShapeLargeTriMesh,
	kPfxShapeReserved0,
	kPfxShapeReserved1,
	kPfxShapeReserved2,
	kPfxShapeUser0,
	kPfxShapeUser1,
	kPfxShapeUser2,
	kPfxShapeCount // =12
	*/

extern void debugRenderPhysics();

static void render(void)
{

	render_begin();
	
	for(int i=0;i<physics_get_num_rigidbodies();i++) {
		const PfxRigidState &state = physics_get_state(i);
		const PfxCollidable &coll = physics_get_collidable(i);

		PfxTransform3 rbT(state.getOrientation(), state.getPosition());

		PfxShapeIterator itrShape(coll);
		for(int j=0;j<coll.getNumShapes();j++,++itrShape) {
			const PfxShape &shape = *itrShape;
			PfxTransform3 offsetT = shape.getOffsetTransform();
			PfxTransform3 worldT = rbT * offsetT;

			switch(shape.getType()) {
				case kPfxShapeSphere:
				render_sphere(
					worldT,
					Vectormath::Aos::Vector3(1,1,1),
					Vectormath::floatInVec(shape.getSphere().m_radius));
				break;

				case kPfxShapeBox:
				render_box(
					worldT,
					Vectormath::Aos::Vector3(1,1,1),
					shape.getBox().m_half);
				break;

				case kPfxShapeCapsule:
				render_capsule(
					worldT,
					Vectormath::Aos::Vector3(1,1,1),
					Vectormath::floatInVec(shape.getCapsule().m_radius),
					Vectormath::floatInVec(shape.getCapsule().m_halfLen));
				break;

				case kPfxShapeCylinder:
				render_cylinder(
					worldT,
					Vectormath::Aos::Vector3(1,1,1),
					Vectormath::floatInVec(shape.getCylinder().m_radius),
					Vectormath::floatInVec(shape.getCylinder().m_halfLen));
				break;

				case kPfxShapeConvexMesh:
				render_mesh(
					worldT,
					Vectormath::Aos::Vector3(1,1,1),
					convexMeshId);
				break;

				case kPfxShapeLargeTriMesh:
				render_mesh(
					worldT,
					Vectormath::Aos::Vector3(1,1,1),
					landscapeMeshId);
				break;

				default:
				break;
			}
		}
	}

	debugRenderPhysics();

	render_end();

}

static int init(void)
{
	perf_init();
	ctrl_init();
	render_init();
	physics_init();

	landscapeMeshId = render_init_mesh(
		LargeMeshVtx,sizeof(float)*6,
		LargeMeshVtx+3,sizeof(float)*6,
		LargeMeshIdx,sizeof(unsigned short)*3,
		LargeMeshVtxCount,LargeMeshIdxCount/3);

	convexMeshId = render_init_mesh(
		BarrelVtx,sizeof(float)*6,
		BarrelVtx+3,sizeof(float)*6,
		BarrelIdx,sizeof(unsigned short)*3,
		BarrelVtxCount,BarrelIdxCount/3);

	return 0;
}

static int shutdown(void)
{
	ctrl_release();
	render_release();
	physics_release();
	perf_release();

	return 0;
}

static void update(void)
{
	static btClock clock;
	static bool first = true;
	btScalar dt=1.f;
	if (first)
	{
		first=false;
		clock.reset();
	} else
	{
		dt = (btScalar)clock.getTimeMicroseconds()/100000.f;
		clock.reset();
	}
	float angX,angY,r=1.f;
	render_get_view_angle(angX,angY,r);

	ctrl_update();
	
	if(ctrl_button_pressed(BTN_UP)) {
		angX -= 0.05f*dt;
		if(angX < -1.4f) angX = -1.4f;
		if(angX > -0.01f) angX = -0.01f;
	}

	if(ctrl_button_pressed(BTN_DOWN)) {
		angX += 0.05f*dt;
		if(angX < -1.4f) angX = -1.4f;
		if(angX > -0.01f) angX = -0.01f;
	}

	if(ctrl_button_pressed(BTN_LEFT)) {
		angY -= 0.05f*dt;
	}

	if(ctrl_button_pressed(BTN_RIGHT)) {
		angY += 0.05f*dt;
	}

	if(ctrl_button_pressed(BTN_ZOOM_OUT)) {
		r *= (1.f+0.2*dt);
		if(r > 500.0f) r = 500.0f;
	}

	if(ctrl_button_pressed(BTN_ZOOM_IN)) {
		r *= (1.f-0.2*dt);
		if(r < 1.0f) r = 1.0f;
	}

	if(ctrl_button_pressed(BTN_SCENE_RESET) == BTN_STAT_DOWN) {
		physics_create_scene(sceneId);
	}

	if(ctrl_button_pressed(BTN_SCENE_NEXT) == BTN_STAT_DOWN) {
		physics_create_scene(++sceneId);
	}

	if(ctrl_button_pressed(BTN_SIMULATION) == BTN_STAT_DOWN) {
		simulating = !simulating;
	}

	if(ctrl_button_pressed(BTN_STEP) == BTN_STAT_DOWN) {
		simulating = true;
	}
	else if(ctrl_button_pressed(BTN_STEP) == BTN_STAT_UP || ctrl_button_pressed(BTN_STEP) == BTN_STAT_KEEP) {
		simulating = false;
	}

	render_set_view_angle(angX,angY,r);
}

#ifndef WIN32

///////////////////////////////////////////////////////////////////////////////
// Main

int main(void)
{
	init();

	physics_create_scene(sceneId);
	
	printf("## %s: INIT SUCCEEDED ##\n", SAMPLE_NAME);

	while (s_isRunning) {
		update();
		if(simulating) physics_simulate();
		render();

		perf_sync();
	}

	shutdown();

	printf("## %s: FINISHED ##\n", SAMPLE_NAME);

	return 0;
}

#else


#include <stdio.h>
#include <setjmp.h>
 
static jmp_buf buf;
 
void second(void) {
    printf("second\n");         // prints
    longjmp(buf,1);             // jumps back to where setjmp was called - making setjmp now return 1
}
 
void first(void) {
    second();
    printf("first\n");          // does not print
}
 
int main2() {   
    if ( ! setjmp(buf) ) {
        first();                // when executed, setjmp returns 0
    } else {                    // when longjmp jumps back, setjmp returns 1
        printf("main\n");       // prints
    }
 
    return 0;
}


///////////////////////////////////////////////////////////////////////////////
// WinMain

extern HDC hDC;
extern HGLRC hRC;
HWND hWnd;
HINSTANCE hInstance;

void releaseWindow()
{
	if(hRC) {
		wglMakeCurrent(0,0);
		wglDeleteContext(hRC);
	}
	
	if(hDC) ReleaseDC(hWnd,hDC);
	if(hWnd) DestroyWindow(hWnd);
	
	UnregisterClass(SAMPLE_NAME,hInstance);
}

LRESULT CALLBACK WndProc(HWND hWnd,UINT	uMsg,WPARAM	wParam,LPARAM lParam)
{
	switch(uMsg) {
		case WM_SYSCOMMAND:
		{
			switch (wParam) {
				case SC_SCREENSAVE:
				case SC_MONITORPOWER:
				return 0;
			}
			break;
		}

		case WM_CLOSE:
		PostQuitMessage(0);
		return 0;

		case WM_SIZE:
		render_resize(LOWORD(lParam),HIWORD(lParam));
		return 0;
	}

	return DefWindowProc(hWnd,uMsg,wParam,lParam);
}

bool createWindow(char* title, int width, int height)
{
	WNDCLASS wc;
	RECT rect;
	rect.left=0;
	rect.right=width;
	rect.top=0;
	rect.bottom=height;

	hInstance = GetModuleHandle(NULL);
	wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wc.lpfnWndProc = (WNDPROC) WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = hInstance;
	wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = NULL;
	wc.lpszMenuName = NULL;
	wc.lpszClassName = SAMPLE_NAME;

	if(!RegisterClass(&wc)) {
		return false;
	}

	AdjustWindowRectEx(&rect, WS_OVERLAPPEDWINDOW, FALSE, WS_EX_APPWINDOW | WS_EX_WINDOWEDGE);

	if(!(hWnd=CreateWindowEx(WS_EX_APPWINDOW|WS_EX_WINDOWEDGE,SAMPLE_NAME,title,
							WS_OVERLAPPEDWINDOW|WS_CLIPSIBLINGS|WS_CLIPCHILDREN,
							0,0,rect.right-rect.left,rect.bottom-rect.top,
							NULL,NULL,hInstance,NULL))) {
		releaseWindow();
		return false;
	}

    static PIXELFORMATDESCRIPTOR pfd = {
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		32,
		0, 0,
		0, 0,
		0, 0,
		0, 0,
		0,
		0, 0, 0, 0,
		32,
		0,
		0,
		PFD_MAIN_PLANE,
		0,
		0, 0, 0
    };
	
	if(!(hDC=GetDC(hWnd)))
	{
		releaseWindow();
		OutputDebugString("");
		return FALSE;
	}
	
	int pixelformat;
	
    if ( (pixelformat = ChoosePixelFormat(hDC, &pfd)) == 0 ){
		OutputDebugString("ChoosePixelFormat Failed....");
        return FALSE;
    }

    if (SetPixelFormat(hDC, pixelformat, &pfd) == FALSE){
		OutputDebugString("SetPixelFormat Failed....");
        return FALSE;
    }

	if (!(hRC=wglCreateContext(hDC))){
		OutputDebugString("Creating HGLRC Failed....");
		return FALSE;
	}
	
	// Set Vsync
	//BOOL (WINAPI *wglSwapIntervalEXT)(int) = NULL;

	//if(strstr((char*)glGetString( GL_EXTENSIONS ),"WGL_EXT_swap_control")== 0) {
	//}
	//else {
		//wglSwapIntervalEXT = (BOOL (WINAPI*)(int))wglGetProcAddress("wglSwapIntervalEXT");
		//if(wglSwapIntervalEXT) wglSwapIntervalEXT(1);
	//}
	
	wglMakeCurrent(hDC,hRC);
	
	ShowWindow(hWnd,SW_SHOW);
	SetForegroundWindow(hWnd);
	SetFocus(hWnd);

	render_resize(width, height);
	
	glClearColor(0.0f,0.0f,0.0f,0.0f);
	glClearDepth(1.0f);
	
	return TRUE;
}

int WINAPI WinMain(HINSTANCE hInstance,HINSTANCE hPrevInstance,LPSTR lpCmdLine,int nCmdShow)
{
	main2();

	if(!createWindow(SAMPLE_NAME,DISPLAY_WIDTH,DISPLAY_HEIGHT)) {
		MessageBox(NULL,"Can't create gl window.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return 0;
	}
	
	init();
	
	physics_create_scene(sceneId);
	
	SCE_PFX_PRINTF("## %s: INIT SUCCEEDED ##\n", SAMPLE_NAME);
	
	MSG msg;
	while(s_isRunning) {
		if(PeekMessage(&msg,NULL,0,0,PM_REMOVE)) {
			if(msg.message==WM_QUIT) {
				s_isRunning = false;
			}
			else {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		else {
			update();
			if(simulating) physics_simulate();
			render();

			perf_sync();
		}
	}

	shutdown();

	SCE_PFX_PRINTF("## %s: FINISHED ##\n", SAMPLE_NAME);

	releaseWindow();
	return (msg.wParam);
}

#endif
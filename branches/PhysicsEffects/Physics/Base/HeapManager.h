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

#ifndef __HEAP_MANAGER_H__
#define __HEAP_MANAGER_H__

// プールされたメモリを管理するスタックのサイズ
#ifdef __SPU__
	#define HEAP_STACK_SIZE 32
#else
	#define HEAP_STACK_SIZE 64
#endif

#define MIN_ALLOC_SIZE 16

//#define HEAP_MANAGER_DEBUG
#ifdef HEAP_MANAGER_DEBUG
	#include <string.h>
#endif

///////////////////////////////////////////////////////////////////////////////
// HeapManager

// ＜補足＞
// メモリはスタックで管理されています。取得した順と逆に開放する必要があります。
// メモリを一気に開放したい場合はclear()を呼び出してください。
// 
// HEAP_MANAGER_DEBUGを定義することでメモリの取得や開放を常にチェックします。
// このとき、チェックが失敗した場合はアサートします。
// 
// 最小割り当てサイズはMIN_ALLOC_SIZEで定義されます。

class HeapManager
{
private:
	unsigned char *mHeap		__attribute__((aligned(16)));
	unsigned int mHeapBytes		__attribute__((aligned(16)));
	unsigned char *mPoolStack[HEAP_STACK_SIZE]		__attribute__((aligned(16)));
	unsigned int mCurStack		__attribute__((aligned(16)));
	
public:
	enum {ALIGN16,ALIGN128};

	HeapManager(unsigned char *buf,int bytes)
	{
		mHeap = buf;
		mHeapBytes = bytes;
		clear();
	}
	
	~HeapManager()
	{
	}
	
	int getAllocated()
	{
		return (int)(mPoolStack[mCurStack]-mHeap);
	}
	
	int getRest()
	{
		return mHeapBytes-getAllocated();
	}

#ifdef HEAP_MANAGER_DEBUG

	void *allocate(size_t bytes,int alignment = ALIGN16)
	{
		if(bytes <= 0) bytes = MIN_ALLOC_SIZE;
		if(mCurStack == HEAP_STACK_SIZE-1) {
			PRINTF("Heap : stack overflow\n");
			PFX_HALT();
		}

		unsigned int p = (unsigned int)mPoolStack[mCurStack];

		if(alignment == ALIGN128) {
			p = ((p+127) & 0xffffff80);
			bytes = (bytes+127) & 0xffffff80;
		}
		else {
			bytes = (bytes+15) & 0xfffffff0;
		}

		if( bytes > (mHeapBytes-(p-(unsigned int)mHeap)) ) {
			PRINTF("Heap : overflow %d bytes\n",bytes);
			PFX_HALT();
		}

		mPoolStack[++mCurStack] = (unsigned char *)(p + bytes);
		PRINTF("Heap : allocate addr:0x%x size:%d (rest %d bytes)\n",(uint32_t)p,bytes,getRest());
		return (void*)p;
	}

	void deallocate(void *p)
	{
		if(mCurStack == 0) {
			PRINTF("Heap : deallocate error! stack is empty\n");
			PFX_HALT();
		}
		unsigned int top = (unsigned int)mPoolStack[mCurStack];
		mCurStack--;
		unsigned int addr = (unsigned int)mPoolStack[mCurStack];
		if(addr != (unsigned int)p && ((addr+127) & 0xffffff80) != (unsigned int)p) {
			PRINTF("Heap : deallocate error! invalid address:0x%x (stack 0x%x)\n",(uint32_t)p,(uint32_t)mPoolStack[mCurStack]);
			PFX_HALT();
		}
		else {
			memset((void*)addr,0,top-addr);
			PRINTF("Heap : deallocate addr:0x%x (rest %d bytes)\n",(uint32_t)p,getRest());
		}
	}

	void clear()
	{
		PRINTF("Heap : clear\n");
		memset(mHeap,0,sizeof(mHeap));
		mPoolStack[0] = mHeap;
		mCurStack = 0;
	}

#else

	void *allocate(size_t bytes,int alignment = ALIGN16)
	{
		if(bytes <= 0) bytes = MIN_ALLOC_SIZE;
		if(mCurStack == HEAP_STACK_SIZE-1) {
			PRINTF("HMERR1\n");
			PFX_HALT();
		}

		unsigned int p = (unsigned int)mPoolStack[mCurStack];

		if(alignment == ALIGN128) {
			p = ((p+127) & 0xffffff80);
			bytes = (bytes+127) & 0xffffff80;
		}
		else {
			bytes = (bytes+15) & 0xfffffff0;
		}

		if( bytes > (mHeapBytes-(p-(unsigned int)mHeap)) ) {
			PRINTF("HMERR2\n");
			PFX_HALT();
		}

		mPoolStack[++mCurStack] = (unsigned char *)(p + bytes);
		return (void*)p;
	}

	void deallocate(void *p)
	{
#if 0
		mCurStack--;
		unsigned int addr = (unsigned int)mPoolStack[mCurStack];
		if(addr != (unsigned int)p && ((addr+127) & 0xffffff80) != (unsigned int)p) {
			PRINTF("HMERR3 0x%x\n",(uint32_t)p);
			PFX_HALT();
		}
#else
		(void) p;
		mCurStack--;
#endif
	}
	
	void clear()
	{
		mPoolStack[0] = mHeap;
		mCurStack = 0;
	}

#endif

	void printStack()
	{
		for(unsigned int i=0;i<=mCurStack;i++) {
			PRINTF("memStack %2d 0x%x\n",i,(uint32_t)mPoolStack[i]);
		}
	}
};

#endif

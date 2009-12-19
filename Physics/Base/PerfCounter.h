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

#ifndef __PERFCOUNTER_H__
#define __PERFCOUNTER_H__

#ifdef WIN32
	#include <windows.h>
#elif defined(__PPU__)
	#include <sys/sys_time.h>
#endif

#include <assert.h>
#include <string.h>

#define MAX_PERF_STR	32
#define MAX_PERF_COUNT	20

#if defined(__PPU__) || defined(WIN32)
	#define USE_PERFCOUNTER
#endif

//#define USE_SPU_BOOKMARK

#ifdef USE_PERFCOUNTER

class PerfCounter
{
private:
	int   mCount,mStrCount;
	char  mStr[MAX_PERF_COUNT][MAX_PERF_STR];
	float mFreq;

#ifdef WIN32
	LONGLONG  mCnt[MAX_PERF_COUNT];
#else
	#ifdef __SPU__
		unsigned long mCnt[MAX_PERF_COUNT];
		unsigned long clockInitEx() {
			return spu_readch(SPU_RdDec);
		}
	#else
		unsigned long mCnt[MAX_PERF_COUNT];
		unsigned long clockInitEx() {
			unsigned long result;

			__asm __volatile__( "mftb %0" : "=r" (result) : : "memory");
			return result;
		}
	#endif
#endif

	void count(int i)
	{
		assert(i < MAX_PERF_COUNT);

#ifdef WIN32
		QueryPerformanceCounter( (LARGE_INTEGER *)&mCnt[i] );
#else
		mCnt[i] = clockInitEx();
#endif
	}

public:
	PerfCounter()
	{
#ifdef WIN32
		LARGE_INTEGER sPerfCountFreq;
		QueryPerformanceFrequency(&sPerfCountFreq);
		mFreq = (float)sPerfCountFreq.QuadPart;
#else
			mFreq = sys_time_get_timebase_frequency();
#endif
		resetCount();
	}

	~PerfCounter()
	{
		printCount();
	}

	void countBegin(const char *name)
	{
		strncpy(mStr[mStrCount],name,MAX_PERF_STR-1);
		mStr[mStrCount][MAX_PERF_STR-1] = '\0';
		mStrCount++;
		count(mCount++);
	}
	
	void countEnd()
	{
		count(mCount++);
	}

	void resetCount()
	{
		mStrCount = 0;
		mCount = 0;
	}

	float getCountTime(int i)
	{
		return (float)(mCnt[i+1]-mCnt[i]) / mFreq * 1000.0f;
	}

	void printCount()
	{
		if(mCount%2 != 0) countEnd();
		PRINTF("*** PerfCounter results ***\n");
		float total = 0.0f;
		for(int i=0;i+1<mCount;i+=2) {
			total += getCountTime(i);
		}
		for(int i=0;i+1<mCount;i+=2) {
//			PRINTF(" -- %s %fms(%.2f%%)\n",mStr[i>>1],getCountTime(i),getCountTime(i)/total*100.0f);
//			PRINTF(" -- %s %f\n",mStr[i>>1],getCountTime(i));
			PRINTF("%s	%f\n",mStr[i>>1],getCountTime(i),getCountTime(i)/total*100.0f);
		}
		PRINTF(" -- Total %fms\n",total);
	}
};

#else /* USE_PERFCOUNTER */

class PerfCounter
{
public:
	PerfCounter() {}
	~PerfCounter() {}
	void countBegin(const char *name) {(void) name;}
	void countEnd() {}
	void resetCount() {}
	float getCountTime(int i) {(void)i;return 0.0f;}
	void printCount() {}
};

#endif /* USE_PERFCOUNTER */

#if defined(__SPU__) && defined(USE_SPU_BOOKMARK)
static inline void insert_bookmark(uint32_t bookmark)
{
	__asm__ volatile ("wrch $69, %0" :: "r" (bookmark));
	__asm__ volatile ("nop;nop;nop;nop;nop;nop;nop;nop");
	__asm__ volatile ("nop;nop;nop;nop;nop;nop;nop;nop");
}
#else
	#define insert_bookmark(bookmark)
#endif

#define BOOKMARK_01 0x7c00
#define BOOKMARK_02 0x03e0
#define BOOKMARK_03 0x001f
#define BOOKMARK_04 0x3c00
#define BOOKMARK_05 0x01e0
#define BOOKMARK_06 0x000f

#ifdef __SPU__
static inline uint32_t getStackSize()
{
	register vec_uint4 sp __asm__("$sp");
	return (uint32_t)spu_extract(sp, 1);
}
#endif

#endif /* __PERFCOUNTER_H__ */

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

#ifndef __QUICKSORT_PPU_H__
#define __QUICKSORT_PPU_H__

#include "Physics/Base/SortCommon.h"

static void quickSort(SortData *x, int left, int right)
{
	if(right - left < 1) return;

	int i, j;
    uint32_t pivot;

    i = left;
    j = right;

    pivot = Key(x[(left + right) / 2]);

    while(1) {
		while (Key(x[i]) < pivot)
            i++;

        while (pivot < Key(x[j]))
            j--;
        if (i >= j)
            break;

		SortData t = x[i];
		x[i] = x[j];
		x[j] = t;
		
		i++;
        j--;
    }

	int num0 = (i - 1) - left;
	int num1 = right - (j + 1);

	if(num0 > num1) {
		if(num1 > 0) quickSort(x, j + 1, right);
		if(num0 > 0) quickSort(x, left, i - 1);
	}
	else {
		if(num0 > 0) quickSort(x, left, i - 1);
		if(num1 > 0) quickSort(x, j + 1, right);
	}
}

#endif /* __QUICKSORT_PPU_H__ */

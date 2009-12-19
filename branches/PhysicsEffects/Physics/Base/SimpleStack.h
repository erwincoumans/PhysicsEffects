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

#ifndef __SIMPLE_STACK_H__
#define __SIMPLE_STACK_H__

#define SIMPLE_STACK_COUNT 20

template <class StackData>
class SimpleStack
{
private:
	uint32_t cur;
	StackData stacks[SIMPLE_STACK_COUNT];

public:
	SimpleStack()
	{
		cur = 0;
	}
	
	void push(const StackData &stack)
	{
		stacks[cur++] = stack;
		PFX_ASSERT(cur<SIMPLE_STACK_COUNT);
	}
	
	StackData pop()
	{
		return stacks[--cur];
	}

	bool isEmpty()
	{
		return cur == 0;
	}

	int getStackCount()
	{
		return cur;
	}
};

#endif // __SIMPLE_STACK_H__

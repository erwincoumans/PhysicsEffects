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

/*
	Collision Func Table
				FIX			ACTIVE		ACTIVE(S)	KEYFRAME	KEYFRAME(S)	ONEWAY		ONEWAY(S)	TRIGGER
	FIX			No			Yes			No			No			No			Yes			Yes			Yes
	ACTIVE		Yes			Yes			Yes			Yes			Yes			Yes			Yes			Yes
	ACTIVE(S)	No			Yes			No			Yes			No			Yes			No			Yes
	KEYFRAME	No			Yes			Yes			No			No			Yes			Yes			Yes
	KEYFRAME(S)	No			Yes			No			No			No			Yes			No			Yes
	ONEWAY		Yes			Yes			Yes			Yes			Yes			Yes			Yes			Yes
	ONEWAY(S)	No			Yes			No			Yes			No			Yes			No			Yes
	TRIGGER		Yes			Yes			Yes			Yes			Yes			Yes			Yes			No
*/

static bool collidableTable[MoveTypeCount][MoveTypeCount] __attribute__ ((aligned(16))) = {
	{false,	true,	false,	false,	false,	true,	true,	true },
	{true,	true,	true,	true,	true,	true,	true,	true },
	{false,	true,	false,	true,	false,	true,	false,	true },
	{false,	true,	true,	false,	false,	true,	true,	true },
	{false,	true,	false,	false,	false,	true,	false,	true },
	{true,	true,	true,	true,	true,	true,	true,	true },
	{false,	true,	false,	true,	false,	true,	false,	true },
	{true,	true,	true,	true,	true,	true,	true,	false},
};

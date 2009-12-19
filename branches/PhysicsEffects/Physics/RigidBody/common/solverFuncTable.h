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
	Response Func Table
				FIX			ACTIVE		ACTIVE(S)	KEYFRAME	KEYFRAME(S)	ONEWAY		ONEWAY(S)	TRIGGER
	FIX			FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	ACTIVE		MovAndFix	MovAndMov	MovAndFix	MovAndFix	MovAndFix	FixAndMov	FixAndFix	FixAndFix
	ACTIVE(S)	FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	KEYFRAME	FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	KEYFRAME(S)	FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	ONEWAY		MovAndFix	MovAndFix	MovAndFix	MovAndFix	MovAndFix	FixAndFix	FixAndFix	FixAndFix
	ONEWAY(S)	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix
	TRIGGER		FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix
*/

static PreResponse funcTbl_preResponse[MoveTypeCount][MoveTypeCount] = {
	{preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseMovAndFix,preResponseMovAndMov,preResponseMovAndFix,preResponseMovAndFix,preResponseMovAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseMovAndFix,preResponseMovAndFix,preResponseMovAndFix,preResponseMovAndFix,preResponseMovAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix},
};

static ApplyImpulse funcTbl_applyImpulse[MoveTypeCount][MoveTypeCount] = {
	{applyImpulseFixAndFix,applyImpulseFixAndMov,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndMov,applyImpulseFixAndFix,applyImpulseFixAndFix},
	{applyImpulseMovAndFix,applyImpulseMovAndMov,applyImpulseMovAndFix,applyImpulseMovAndFix,applyImpulseMovAndFix,applyImpulseFixAndMov,applyImpulseFixAndFix,applyImpulseFixAndFix},
	{applyImpulseFixAndFix,applyImpulseFixAndMov,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndMov,applyImpulseFixAndFix,applyImpulseFixAndFix},
	{applyImpulseFixAndFix,applyImpulseFixAndMov,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndMov,applyImpulseFixAndFix,applyImpulseFixAndFix},
	{applyImpulseFixAndFix,applyImpulseFixAndMov,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndMov,applyImpulseFixAndFix,applyImpulseFixAndFix},
	{applyImpulseMovAndFix,applyImpulseMovAndFix,applyImpulseMovAndFix,applyImpulseMovAndFix,applyImpulseMovAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix},
	{applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix},
	{applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix,applyImpulseFixAndFix},
};

/*
	Joint Func Table
				FIX			ACTIVE		ACTIVE(S)	KEYFRAME	KEYFRAME(S)	ONEWAY		ONEWAY(S)	TRIGGER
	FIX			FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	ACTIVE		MovAndFix	MovAndMov	MovAndMov	MovAndFix	MovAndFix	FixAndMov	FixAndMov	FixAndFix
	ACTIVE(S)	FixAndFix	MovAndMov	FixAndFix	MovAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	KEYFRAME	FixAndFix	FixAndMov	FixAndMov	FixAndFix	FixAndFix	FixAndMov	FixAndMov	FixAndFix
	KEYFRAME(S)	FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	ONEWAY		MovAndFix	MovAndFix	MovAndFix	MovAndFix	MovAndFix	MovAndMov	MovAndMov	FixAndFix
	ONEWAY(S)	FixAndFix	MovAndFix	FixAndFix	MovAndFix	FixAndFix	MovAndMov	FixAndFix	FixAndFix
	TRIGGER		FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix
*/

static PreJoint funcTbl_preJoint[MoveTypeCount][MoveTypeCount] = {
	{preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix},
	{preJointMovAndFix,preJointMovAndMov,preJointMovAndMov,preJointMovAndFix,preJointMovAndFix,preJointFixAndMov,preJointFixAndMov,preJointFixAndFix},
	{preJointFixAndFix,preJointMovAndMov,preJointFixAndFix,preJointMovAndFix,preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix},
	{preJointFixAndFix,preJointFixAndMov,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix,preJointFixAndMov,preJointFixAndMov,preJointFixAndFix},
	{preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix},
	{preJointMovAndFix,preJointMovAndFix,preJointMovAndFix,preJointMovAndFix,preJointMovAndFix,preJointMovAndMov,preJointMovAndMov,preJointFixAndFix},
	{preJointFixAndFix,preJointMovAndFix,preJointFixAndFix,preJointMovAndFix,preJointFixAndFix,preJointMovAndMov,preJointFixAndFix,preJointFixAndFix},
	{preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix},
};

static ApplyJoint funcTbl_applyJoint[MoveTypeCount][MoveTypeCount] = {
	{applyJointFixAndFix,applyJointFixAndMov,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndMov,applyJointFixAndFix,applyJointFixAndFix},
	{applyJointMovAndFix,applyJointMovAndMov,applyJointMovAndMov,applyJointMovAndFix,applyJointMovAndFix,applyJointFixAndMov,applyJointFixAndMov,applyJointFixAndFix},
	{applyJointFixAndFix,applyJointMovAndMov,applyJointFixAndFix,applyJointMovAndFix,applyJointFixAndFix,applyJointFixAndMov,applyJointFixAndFix,applyJointFixAndFix},
	{applyJointFixAndFix,applyJointFixAndMov,applyJointFixAndMov,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndMov,applyJointFixAndMov,applyJointFixAndFix},
	{applyJointFixAndFix,applyJointFixAndMov,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndMov,applyJointFixAndFix,applyJointFixAndFix},
	{applyJointMovAndFix,applyJointMovAndFix,applyJointMovAndFix,applyJointMovAndFix,applyJointMovAndFix,applyJointMovAndMov,applyJointMovAndMov,applyJointFixAndFix},
	{applyJointFixAndFix,applyJointMovAndFix,applyJointFixAndFix,applyJointMovAndFix,applyJointFixAndFix,applyJointMovAndMov,applyJointFixAndFix,applyJointFixAndFix},
	{applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndFix,applyJointFixAndFix},
};

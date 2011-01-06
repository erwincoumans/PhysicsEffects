/* SCE CONFIDENTIAL
 * PSP2 Programmer Tool Runtime Library Release 00.930.000
 *                Copyright (C) 2010 Sony Computer Entertainment Inc.
 *                                                All Rights Reserved.
 */

#ifndef __PHYSICS_FUNC_H__
#define __PHYSICS_FUNC_H__

#define SCE_PFX_USE_PERFCOUNTER

#include "physics_effects.h"

using namespace sce::PhysicsEffects;

//E Simulation
//J シミュレーション
bool physics_init();
void physics_release();
void physics_create_scene(int sceneId);
void physics_simulate();

//E Picking
//J ピッキング
PfxVector3 physics_pick_start(const PfxVector3 &p1,const PfxVector3 &p2);
void physics_pick_update(const PfxVector3 &p);
void physics_pick_end();

//E Change parameters
//J パラメータの取得
int physics_get_num_rigidbodies();
const PfxRigidState& physics_get_state(int id);
const PfxRigidBody& physics_get_body(int id);
const PfxCollidable& physics_get_collidable(int id);

int physics_get_num_contacts();
const PfxContactManifold &physics_get_contact(int id);

#endif /* __PHYSICS_FUNC_H__ */

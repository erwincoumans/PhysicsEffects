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

#include "Physics/RigidBody/common/Contact.h"

int findNearestContactPoint(ContactPoint *cp,int numContacts,const Vector3 &newCP)
{
	int nearestIdx = -1;
	for(int i=0;i<numContacts;i++) {
		Vector3 dist = cp[i].getLocalPointA()-newCP;
		float diff = lengthSqr(dist);
		if(diff < CONTACT_THRESHOLD_TANGENT) {
			nearestIdx = i;
		}
	}
	return nearestIdx;
}

int  sort4ContactPoints(ContactPoint *cp,const Vector3 &newCP,float newDistance)
{
	int maxPenetrationIndex = -1;
	float maxPenetration = newDistance;

	// 最も深い衝突点は排除対象からはずす
	for(int i=0;i<NUMCONTACTS_PER_BODIES;i++) {
		if(cp[i].distance < maxPenetration) {
			maxPenetrationIndex = i;
			maxPenetration = cp[i].distance;
		}
	}
	
	float res[4] = {0.0f};
	
	// 各点を除いたときの衝突点が作る面積のうち、最も大きくなるものを選択
	if(maxPenetrationIndex != 0) {
		Vector3 a0 = newCP-cp[1].getLocalPointA();
		Vector3 b0 = cp[3].getLocalPointA()-cp[2].getLocalPointA();
		res[0] = lengthSqr(cross(a0,b0));
	}
 
	if(maxPenetrationIndex != 1) {
		Vector3 a1 = newCP-cp[0].getLocalPointA();
		Vector3 b1 = cp[3].getLocalPointA()-cp[2].getLocalPointA();
		res[1] = lengthSqr(cross(a1,b1));
	}

	if(maxPenetrationIndex != 2) {
		Vector3 a2 = newCP-cp[0].getLocalPointA();
		Vector3 b2 = cp[3].getLocalPointA()-cp[1].getLocalPointA();
		res[2] = lengthSqr(cross(a2,b2));
	}

	if(maxPenetrationIndex != 3) {
		Vector3 a3 = newCP-cp[0].getLocalPointA();
		Vector3 b3 = cp[2].getLocalPointA()-cp[1].getLocalPointA();
		res[3] = lengthSqr(cross(a3,b3));
	}

	int maxIndex = 0;
	float maxVal = res[0];

	if (res[1] > maxVal) {
		maxIndex = 1;
		maxVal = res[1];
	}

	if (res[2] > maxVal) {
		maxIndex = 2;
		maxVal = res[2];
	}

	if (res[3] > maxVal) {
		maxIndex = 3;
		maxVal = res[3];
	}

	return maxIndex;
}

uint32_t ContactPair::merge(ContactPair &contactPair)
{
	if(stateIndex[0] != contactPair.stateIndex[0])
		contactPair.exchange();

	uint32_t ret = 0;

	for(int i=0;i<contactPair.numContacts;i++) {
		int idx = findNearestContactPoint(contactPoints,numContacts,contactPair.contactPoints[i].getLocalPointA());

		if(idx >= 0) {
			if(contactPoints[idx].distance > contactPair.contactPoints[i].distance) {
				// 同一点を発見、蓄積された情報を継続
				uint8_t d = contactPoints[idx].duration;
				float accumN  = contactPoints[idx].constraints[0].accumImpulse;
				float accumF1 = contactPoints[idx].constraints[1].accumImpulse;
				float accumF2 = contactPoints[idx].constraints[2].accumImpulse;
				contactPoints[idx] = contactPair.contactPoints[i];
				contactPoints[idx].constraints[0].accumImpulse = accumN ;
				contactPoints[idx].constraints[1].accumImpulse = accumF1;
				contactPoints[idx].constraints[2].accumImpulse = accumF2;
				contactPoints[idx].duration = d;
			}
			ret |= 1; // 継続
			continue;
		}

		if(numContacts < NUMCONTACTS_PER_BODIES) {
			// 衝突点を新規追加
			contactPoints[numContacts++] = contactPair.contactPoints[i];
			ret |= 2; // 新規
		}
		else {
			// ソート
			int newIdx = sort4ContactPoints(contactPoints,contactPair.contactPoints[i].getLocalPointA(),contactPair.contactPoints[i].distance);
			
			// コンタクトポイント入れ替え
			contactPoints[newIdx] = contactPair.contactPoints[i];
			ret |= 4; // 入替
		}
	}

	return ret;
}

void ContactPair::refreshContactPoints(const Vector3 &pA,const Quat &qA,const Vector3 &pB,const Quat &qB)
{
	// 衝突点の更新
	// 両衝突点間の距離が閾値（CONTACT_THRESHOLD）を超えたら消去
	for(int i=0;i<(int)numContacts;i++) {
		if(contactPoints[i].duration > 0) {
			Vector3 cpA = contactPoints[i].getWorldPointA(pA,qA);
			Vector3 cpB = contactPoints[i].getWorldPointB(pB,qB);

			// 貫通深度がプラスに転じたかどうかをチェック
			contactPoints[i].distance = dot(contactPoints[i].getNormal(),(cpA - cpB));
			if(contactPoints[i].distance > CONTACT_THRESHOLD_NORMAL) {
				removeContactPoint(i);
				i--;
				continue;
			}

			// 深度方向を除去して両点の距離をチェック
			cpA = cpA - contactPoints[i].distance * contactPoints[i].getNormal();
			float distanceAB = lengthSqr(cpA - cpB);
			if(distanceAB > CONTACT_THRESHOLD_TANGENT) {
				removeContactPoint(i);
				i--;
				continue;
			}
		}
		if(contactPoints[i].duration < 255) contactPoints[i].duration++;
	}
	duration++;
}

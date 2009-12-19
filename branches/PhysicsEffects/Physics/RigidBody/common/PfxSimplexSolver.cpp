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
#include <stdio.h>
#include <float.h>

#include "PfxSimplexSolver.h"
#include "PfxGJKSolver.h"

inline static
bool operator ==(const Vector3 &a,const Vector3 &b)
{
	return lengthSqr(a-b) < (PFX_GJK_EPSILON * PFX_GJK_EPSILON);
}

bool PfxSimplexSolver::closest(Vector3& v)
{
	bool ret = false;

	bc.reset();

	switch(numVertices) {
		case 0:
		ret = false;
		break;

		case 1:
		{
			Vector3 tmpP = P[0];
			Vector3 tmpQ = Q[0];
			v = tmpP-tmpQ;
			bc.reset();
			bc.setBarycentricCoordinates(1.0f,0.0f,0.0f,0.0f);
			ret = bc.isValid();
		}
		break;

		case 2:
		{
			Vector3 dir = W[1] - W[0];
			float t = dot(-W[0],dir) / dot(dir,dir);

			if(t < 0.0f) t = 0.0f;
			if(t > 1.0f) t = 1.0f;

			bc.setBarycentricCoordinates(1-t,t,0.0f,0.0f);

			Vector3 tmpP = P[0] + t * (P[1] - P[0]);
			Vector3 tmpQ = Q[0] + t * (Q[1] - Q[0]);
			v = tmpP - tmpQ;

			reduceVertices();

			ret = bc.isValid();
			break;
		}

		case 3: 
		{ 
			const Vector3& a = W[0]; 
			const Vector3& b = W[1]; 
			const Vector3& c = W[2]; 

			closestPointTriangleFromOrigin(a,b,c,bc);

			Vector3 tmpP = P[0] * bc.barycentricCoords[0] + 
						   P[1] * bc.barycentricCoords[1] + 
						   P[2] * bc.barycentricCoords[2]; 

			Vector3 tmpQ = Q[0] * bc.barycentricCoords[0] + 
						   Q[1] * bc.barycentricCoords[1] + 
						   Q[2] * bc.barycentricCoords[2]; 

			v = tmpP-tmpQ; 

			reduceVertices(); 
			ret = bc.isValid(); 
			break; 
		}

		case 4:
		{
			const Vector3& a = W[0];
			const Vector3& b = W[1];
			const Vector3& c = W[2];
			const Vector3& d = W[3];

			if(closestPointTetrahedronFromOrigin(a,b,c,d,bc)) {
				Vector3 tmpP = P[0] * bc.barycentricCoords[0] +
							   P[1] * bc.barycentricCoords[1] +
							   P[2] * bc.barycentricCoords[2] +
							   P[3] * bc.barycentricCoords[3];

				Vector3 tmpQ = Q[0] * bc.barycentricCoords[0] +
							   Q[1] * bc.barycentricCoords[1] +
							   Q[2] * bc.barycentricCoords[2] +
							   Q[3] * bc.barycentricCoords[3];
				v = tmpP-tmpQ;

				reduceVertices();
				ret = bc.isValid();
			} else {
				// 原点が内部に存在→交差している
				ret = true;
				v = Vector3(0.0f);
			}
			break;
		}
	};

	return ret;
}

bool PfxSimplexSolver::inSimplex(const Vector3& w)
{
	for(int i=0;i<numVertices;i++) {
		if(W[i] == w)
			return true;
	}
	return false;
}

bool PfxSimplexSolver::closestPointTriangleFromOrigin(Vector3 a, Vector3 b, Vector3 c,PfxBarycentricCoords& result)
{
	result.usedVertices = 0;
	Vector3 p(0.0f);

    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ap = p - a;
    float d1 = dot(ab,ap);
    float d2 = dot(ac,ap);
    if(d1 <= 0.0f && d2 <= 0.0f) {
		result.closest = a;
		result.setBarycentricCoordinates(1.0f,0.0f,0.0f,0.0f);
		return true;
	}

    Vector3 bp = p - b;
    float d3 = dot(ab,bp);
    float d4 = dot(ac,bp);
    if(d3 >= 0.0f && d4 <= d3) {
		result.closest = b;
		result.setBarycentricCoordinates(0.0f,1.0f,0.0f,0.0f);
		return true;
	}

    float vc = d1*d4 - d3*d2;
    if(vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
		result.closest = a + v * ab;
		result.setBarycentricCoordinates(1.0f-v,v,0.0f,0.0f);
		return true;
    }

    Vector3 cp = p - c;
    float d5 = dot(ab,cp);
    float d6 = dot(ac,cp);
    if(d6 >= 0.0f && d5 <= d6) {
		result.closest = c;
		result.setBarycentricCoordinates(0.0f,0.0f,1.0f,0.0f);
		return true;
	}

    float vb = d5*d2 - d1*d6;
    if(vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
		result.closest = a + w * ac;
		result.setBarycentricCoordinates(1.0f-w,0.0f,w,0.0f);
		return true;
    }

    float va = d3*d6 - d5*d4;
    if(va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		result.closest = b + w * (c - b);
		result.setBarycentricCoordinates(0.0f,1.0f-w,w,0.0f);
		return true;		
    }

    float denom = float(1.0) / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    
	result.closest = a + ab * v + ac * w;
	result.setBarycentricCoordinates(1.0f-v-w,v,w,0.0f);
	
	return true;
}

bool PfxSimplexSolver::closestPointTetrahedronFromOrigin(Vector3 a, Vector3 b, Vector3 c, Vector3 d, PfxBarycentricCoords& finalResult)
{
	PfxBarycentricCoords tempResult;
	Vector3 p(0.0f);

	finalResult.closest = p;
	finalResult.usedVertices = 0;

	bool pointOutsideABC = originOutsideOfPlane(a, b, c, d);
	bool pointOutsideACD = originOutsideOfPlane(a, c, d, b);
	bool pointOutsideADB = originOutsideOfPlane(a, d, b, c);
	bool pointOutsideBDC = originOutsideOfPlane(b, d, c, a);

	if(!pointOutsideABC && !pointOutsideACD && !pointOutsideADB && !pointOutsideBDC)
		return false;

	float bestSqDist = FLT_MAX;

	if(pointOutsideABC) {
		closestPointTriangleFromOrigin(a, b, c,tempResult);
		Vector3 q = tempResult.closest;
		float sqDist = dot((q - p),(q - p));
		if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
			finalResult.setBarycentricCoordinates(
					tempResult.barycentricCoords[0],
					tempResult.barycentricCoords[1],
					tempResult.barycentricCoords[2],
					0);
		}
    }
  
	if(pointOutsideACD) {
		closestPointTriangleFromOrigin(a, c, d,tempResult);
		Vector3 q = tempResult.closest;
        float sqDist = dot((q - p),(q - p));
        if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
			finalResult.setBarycentricCoordinates(
					tempResult.barycentricCoords[0],
					0,
					tempResult.barycentricCoords[1],
					tempResult.barycentricCoords[2]);
		}
    }
	
	if(pointOutsideADB) {
		closestPointTriangleFromOrigin(a, d, b,tempResult);
		Vector3 q = tempResult.closest;
        float sqDist = dot((q - p),(q - p));
        if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
			finalResult.setBarycentricCoordinates(
					tempResult.barycentricCoords[0],
					tempResult.barycentricCoords[2],
					0,
					tempResult.barycentricCoords[1]);
		}
    }

	if(pointOutsideBDC) {
		closestPointTriangleFromOrigin(b, d, c,tempResult);
		Vector3 q = tempResult.closest;
        float sqDist = dot((q - p),(q - p));
        if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
			finalResult.setBarycentricCoordinates(
					0,
					tempResult.barycentricCoords[0],
					tempResult.barycentricCoords[2],
					tempResult.barycentricCoords[1]);
		}
    }

    return true;
}

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


#ifndef __PFX_SIMPLEX_SOLVER_H__
#define __PFX_SIMPLEX_SOLVER_H__

#include "Physics/Base/PhysicsCommon.h"

#include <vectormath_aos.h>
using namespace Vectormath::Aos;


///////////////////////////////////////////////////////////////////////////////
// Voronoi Simplex Solver

struct PfxBarycentricCoords {
	Vector3 closest;
	float barycentricCoords[4];
	unsigned int usedVertices;

	void reset()
	{
		barycentricCoords[0] = 0.0f;
		barycentricCoords[1] = 0.0f;
		barycentricCoords[2] = 0.0f;
		barycentricCoords[3] = 0.0f;
		usedVertices = 0;
	}

	bool isValid()
	{
		return   (barycentricCoords[0] >= 0.0f) &&
				 (barycentricCoords[1] >= 0.0f) &&
				 (barycentricCoords[2] >= 0.0f) &&
				 (barycentricCoords[3] >= 0.0f);
	}

	void setBarycentricCoordinates(float a,float b,float c,float d)
	{
		barycentricCoords[0] = a;
		barycentricCoords[1] = b;
		barycentricCoords[2] = c;
		barycentricCoords[3] = d;
		if(a != 0.0f) usedVertices |= 1<<3;
		if(b != 0.0f) usedVertices |= 1<<2;
		if(c != 0.0f) usedVertices |= 1<<1;
		if(d != 0.0f) usedVertices |= 1;
	}
} __attribute__ ((aligned(16)));

class PfxSimplexSolver {
private:
	const static int MAX_VERTS = 4;

public:
	int	numVertices;
	Vector3	W[MAX_VERTS];
	Vector3	P[MAX_VERTS];
	Vector3	Q[MAX_VERTS];

	PfxBarycentricCoords bc;

	inline void	removeVertex(int index);
	inline void	reduceVertices ();

	inline bool	originOutsideOfPlane(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d);
	bool	closestPointTetrahedronFromOrigin(Vector3 a, Vector3 b, Vector3 c, Vector3 d, PfxBarycentricCoords& result);
	bool	closestPointTriangleFromOrigin(Vector3 a, Vector3 b, Vector3 c,PfxBarycentricCoords& result);

public:
	void reset()
	{
		numVertices = 0;
		bc.reset();
	}

	inline void addVertex(const Vector3& w_, const Vector3& p_, const Vector3& q_);

	bool closest(Vector3& v);

	bool fullSimplex() const
	{
		return (numVertices == 4);
	}

	bool inSimplex(const Vector3& w);
};

inline
void	PfxSimplexSolver::removeVertex(int index)
{
	PFX_ASSERT(numVertices>0);
	numVertices--;
	W[index] = W[numVertices];
	P[index] = P[numVertices];
	Q[index] = Q[numVertices];
}

inline
void	PfxSimplexSolver::reduceVertices ()
{
	if ((numVertices >= 4) && (!(bc.usedVertices&0x01)))
		removeVertex(3);

	if ((numVertices >= 3) && (!(bc.usedVertices&0x02)))
		removeVertex(2);

	if ((numVertices >= 2) && (!(bc.usedVertices&0x04)))
		removeVertex(1);
	
	if ((numVertices >= 1) && (!(bc.usedVertices&0x08)))
		removeVertex(0);
}

inline
void PfxSimplexSolver::addVertex(const Vector3& w, const Vector3& p, const Vector3& q)
{
	W[numVertices] = w;
	P[numVertices] = p;
	Q[numVertices] = q;
	numVertices++;
}

inline
bool PfxSimplexSolver::originOutsideOfPlane(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d)
{
	Vector3 normal = cross((b-a),(c-a));

    float signp = dot(-a,normal);
    float signd = dot((d - a),normal);

	return signp * signd < 0.0f;
}

#endif /* __PFX_SIMPLEX_SOLVER_H__ */

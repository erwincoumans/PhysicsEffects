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
#include <vectormath_aos.h>

#include "Box.h"
#include "Capsule.h"

using namespace Vectormath::Aos;

enum BoxCapsSepAxisType
{
	BOX_AXIS, CROSS_AXIS
};

//-------------------------------------------------------------------------------------------------
// voronoiTol: bevels Voronoi planes slightly which helps when features are parallel.
//-------------------------------------------------------------------------------------------------

static const float voronoiTol = -1.0e-5f;

//-------------------------------------------------------------------------------------------------
// lenSqrTol: minimum square of length for safe normalize.
//-------------------------------------------------------------------------------------------------

static const float lenSqrTol = 1.0e-30f;

//-------------------------------------------------------------------------------------------------
// separating axis tests: gaps along each axis are computed, and the axis with the maximum
// gap is stored.  cross product axes are normalized.
//-------------------------------------------------------------------------------------------------

#define AaxisTest( dim, letter, first )                                                         \
{                                                                                               \
   if ( first )                                                                                 \
   {                                                                                            \
      maxGap = gapsA.get##letter();                                                             \
      if ( maxGap - capsuleB.radius > distanceThreshold ) return maxGap - capsuleB.radius;      \
      axisType = BOX_AXIS;                                                                      \
      faceDimA = dim;                                                                           \
      axisA = identity.getCol##dim();;                                                          \
   }                                                                                            \
   else                                                                                         \
   {                                                                                            \
      float gap = gapsA.get##letter();                                                          \
      if ( gap - capsuleB.radius > distanceThreshold ) return gap - capsuleB.radius;            \
      else if ( gap > maxGap )                                                                  \
      {                                                                                         \
         maxGap = gap;                                                                          \
         axisType = BOX_AXIS;                                                                   \
         faceDimA = dim;                                                                        \
         axisA = identity.getCol##dim();;                                                       \
      }                                                                                         \
   }                                                                                            \
}

#define CrossAxisTest( dima, lettera )                                                          \
{                                                                                               \
   const float lsqr_tolerance = 1.0e-30f;                                                       \
   float lsqr;                                                                                  \
                                                                                                \
   lsqr = lsqrs.get##lettera();                                                                 \
                                                                                                \
   if ( lsqr > lsqr_tolerance )                                                                 \
   {                                                                                            \
      float l_recip = 1.0f / sqrtf( lsqr );                                                     \
      float gap = float(gapsAxB.get##lettera()) * l_recip;                                      \
                                                                                                \
      if ( gap - capsuleB.radius > distanceThreshold )                                          \
      {                                                                                         \
         return gap - capsuleB.radius;                                                          \
      }                                                                                         \
                                                                                                \
      if ( gap > maxGap )                                                                       \
      {                                                                                         \
         maxGap = gap;                                                                          \
         axisType = CROSS_AXIS;                                                                 \
         edgeDimA = dima;                                                                       \
         axisA = crossProdMat.getCol##dima() * l_recip;                                         \
      }                                                                                         \
   }                                                                                            \
}

//-------------------------------------------------------------------------------------------------
// tests whether a vertex of box B and a face of box A are the closest features
//-------------------------------------------------------------------------------------------------

inline
float
VertexBFaceATest(
	bool& inVoronoi,
	float& t0,
	float& t1,
	Vector3& ptsVec,
	const Vector3& hA,
	Vector3 offsetAB,
	Vector3 capsDirection,
	float signB,
	float scaleB )
{
	// compute endpoint of capsule in box's coordinate system

	Vector3 endpoint = Vector3( offsetAB + capsDirection * scaleB );

	// compute the parameters of the point on the box face closest to this corner.

	t0 = endpoint[0];
	t1 = endpoint[1];

	if ( t0 > hA[0] )
		t0 = hA[0];
	else if ( t0 < -hA[0] )
		t0 = -hA[0];
	if ( t1 > hA[1] )
		t1 = hA[1];
	else if ( t1 < -hA[1] )
		t1 = -hA[1];

	// get vector from face point to capsule endpoint

	endpoint[0] -= t0;
	endpoint[1] -= t1;
	ptsVec = Vector3(endpoint);

	// do the Voronoi test: already know the point on B is in the Voronoi region of the
	// point on A, check the reverse.

	inVoronoi = ( -signB * dot(ptsVec,capsDirection) >= voronoiTol );

	return (lengthSqr(ptsVec));
}

#define VertexBFaceA_SetNewMin()                \
{                                               \
   minDistSqr = distSqr;                        \
   closestPtsVec = ptsVec;                      \
   localPointA.setX(t0);                        \
   localPointA.setY(t1);                        \
   segmentParamB = scaleB;                      \
   featureA = F;                                \
   featureB = V;                                \
}

void
VertexBFaceATests(
	bool& done,
	float& minDistSqr,
	Vector3& closestPtsVec,
	Point3& localPointA,
	float& segmentParamB,
	FeatureType & featureA,
	FeatureType & featureB,
	const Vector3& hA,
	Vector3 offsetAB,
	Vector3 capsDirection,
	float signB, float scaleB,
	bool first )
{
	Vector3 ptsVec;
	float t0, t1;
	float distSqr;

	// test endpoint of capsule nearest to face

	distSqr = VertexBFaceATest( done, t0, t1, ptsVec, hA, offsetAB, capsDirection, signB, scaleB );

	if ( first ) {
		VertexBFaceA_SetNewMin();
	} else {
		if ( distSqr < minDistSqr ) {
			VertexBFaceA_SetNewMin();
		}
	}

	if ( done )
		return;

	signB = -signB;
	scaleB = -scaleB;

	// test other endpoint if necessary

	distSqr = VertexBFaceATest( done, t0, t1, ptsVec, hA, offsetAB, capsDirection, signB, scaleB );

	if ( distSqr < minDistSqr ) {
		VertexBFaceA_SetNewMin();
	}
}

//-------------------------------------------------------------------------------------------------
// EdgeEdgeTest:
//
// tests whether a pair of edges are the closest features
//
// note on the shorthand:
// 'a' & 'b' refer to the edges.
// 'c' is the dimension of the axis that points from the face center to the edge Center
// 'd' is the dimension of the edge Direction
// the dimension of the face normal is 2
//-------------------------------------------------------------------------------------------------

#define EdgeEdgeTest( ac, ac_letter, ad, ad_letter )                                            \
{                                                                                               \
   /* get vector between edge centers */                                                        \
                                                                                                \
   ptsVec = offsetAB;                                                                           \
   ptsVec.set##ac_letter( ptsVec.get##ac_letter() - scalesA.get##ac_letter() );                 \
                                                                                                \
   /* find parameters of closest points on line segments. */                                    \
                                                                                                \
   float capsDirection_ad = capsDirection.get##ad_letter();                                     \
   float ptsVec_ad = ptsVec.get##ad_letter();                                                   \
   float capsDirDotPtsVec = dot(capsDirection,ptsVec);                                          \
   float denom = 1.0f - capsDirection_ad * capsDirection_ad;                                    \
                                                                                                \
   if ( denom == 0.0f )                                                                         \
   {                                                                                            \
      tA = 0.0f;                                                                                \
   }                                                                                            \
   else                                                                                         \
   {                                                                                            \
      tA = ( ptsVec_ad - capsDirDotPtsVec * capsDirection_ad ) / denom;                         \
      if ( tA < -hA[ad] ) tA = -hA[ad];                                                         \
      else if ( tA > hA[ad] ) tA = hA[ad];                                                      \
   }                                                                                            \
                                                                                                \
   tB = tA * capsDirection_ad - capsDirDotPtsVec;                                               \
                                                                                                \
   if ( tB < -hB )                                                                              \
   {                                                                                            \
      tB = -hB;                                                                                 \
      tA = tB * capsDirection_ad + ptsVec_ad;                                                   \
                                                                                                \
      if ( tA < -hA[ad] ) tA = -hA[ad];                                                         \
      else if ( tA > hA[ad] ) tA = hA[ad];                                                      \
   }                                                                                            \
   else if ( tB > hB )                                                                          \
   {                                                                                            \
      tB = hB;                                                                                  \
      tA = tB * capsDirection_ad + ptsVec_ad;                                                   \
                                                                                                \
      if ( tA < -hA[ad] ) tA = -hA[ad];                                                         \
      else if ( tA > hA[ad] ) tA = hA[ad];                                                      \
   }                                                                                            \
                                                                                                \
   /* make vector to point at tB on edge B from the center of edge A. */                        \
   /* test that it lies inside edge A's voronoi region. */                                      \
                                                                                                \
   ptsVec += capsDirection * tB;                                                                \
                                                                                                \
   Vector3 cptsVec( mulPerElem( ptsVec, signsA ) );                                             \
                                                                                                \
   inVoronoi = ( cptsVec[ac] >= voronoiTol * cptsVec[2] ) &&                                    \
               ( cptsVec[2] >= voronoiTol * cptsVec[ac] );                                      \
                                                                                                \
   ptsVec.set##ad_letter( ptsVec.get##ad_letter() - tA );                                       \
                                                                                                \
   return lengthSqr(ptsVec);                                                                    \
}

float
EdgeEdgeTest_01(
	bool& inVoronoi,
	float& tA,
	float& tB,
	Vector3& ptsVec,
	const Vector3& hA,
	float hB,
	Vector3 offsetAB,
	Vector3 capsDirection,
	Vector3 signsA,
	Vector3 scalesA )
{
	EdgeEdgeTest( 0, X, 1, Y );
}

float
EdgeEdgeTest_10(
	bool& inVoronoi,
	float& tA,
	float& tB,
	Vector3& ptsVec,
	const Vector3& hA,
	float hB,
	Vector3 offsetAB,
	Vector3 capsDirection,
	Vector3 signsA,
	Vector3 scalesA )
{
	EdgeEdgeTest( 1, Y, 0, X );
}

#define EdgeEdge_SetNewMin( ac_letter, ad_letter )                         \
{                                                                          \
   minDistSqr = distSqr;                                                   \
   closestPtsVec = ptsVec;                                                 \
   localPointA.set##ac_letter(scalesA.get##ac_letter());                   \
   localPointA.set##ad_letter(tA);                                         \
   segmentParamB = tB;                                                     \
   otherFaceDimA = testOtherFaceDimA;                                      \
   featureA = E;                                                           \
   featureB = E;                                                           \
}

void
EdgeEdgeTests(
	bool& done,
	float& minDistSqr,
	Vector3& closestPtsVec,
	Point3& localPointA,
	float& segmentParamB,
	int & otherFaceDimA,
	FeatureType & featureA,
	FeatureType & featureB,
	const Vector3& hA,
	float hB,
	Vector3 offsetAB,
	Vector3 capsDirection,
	Vector3 signsA,
	Vector3 scalesA,
	bool first )
{
	Vector3 ptsVec;
	float tA, tB;
	int testOtherFaceDimA;

	testOtherFaceDimA = 0;

	float distSqr = EdgeEdgeTest_01( done, tA, tB, ptsVec, hA, hB,
									 offsetAB, capsDirection, signsA, scalesA );

	if ( first ) {
		EdgeEdge_SetNewMin( X, Y );
	} else {
		if ( distSqr < minDistSqr ) {
			EdgeEdge_SetNewMin( X, Y );
		}
	}

	if ( done )
		return;

	signsA.setX( -signsA.getX() );
	scalesA.setX( -scalesA.getX() );

	distSqr = EdgeEdgeTest_01( done, tA, tB, ptsVec, hA, hB,
							   offsetAB, capsDirection, signsA, scalesA );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( X, Y );
	}

	if ( done )
		return;

	testOtherFaceDimA = 1;

	distSqr = EdgeEdgeTest_10( done, tA, tB, ptsVec, hA, hB,
							   offsetAB, capsDirection, signsA, scalesA );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X );
	}

	if ( done )
		return;

	signsA.setY( -signsA.getY() );
	scalesA.setY( -scalesA.getY() );

	distSqr = EdgeEdgeTest_10( done, tA, tB, ptsVec, hA, hB,
							   offsetAB, capsDirection, signsA, scalesA );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X );
	}
}

float
boxCapsuleDistance(
	Vector3& normal,
	BoxPoint& boxPointA,
	CapsulePoint& capsulePointB,
	Box boxA, const Transform3& transformA,
	const Capsule& capsuleB, const Transform3& transformB,
	float distanceThreshold )
{
	Matrix3 identity = Matrix3::identity();
	Vector3 ident[3];
	ident[0] = identity.getCol0();
	ident[1] = identity.getCol1();
	ident[2] = identity.getCol2();

	// get capsule position and direction in box's coordinate system

	Matrix3 matrixA = transformA.getUpper3x3();
	Matrix3 matrixAinv = transpose(matrixA);

	Vector3 directionB = transformB.getUpper3x3().getCol0();
	Vector3 translationB = transformB.getTranslation();

	Vector3 capsDirection = matrixAinv * directionB;
	Vector3 absCapsDirection = absPerElem(capsDirection);
	Vector3 offsetAB = matrixAinv * (translationB - transformA.getTranslation());

	// find separating axis with largest gap between projections

	BoxCapsSepAxisType axisType;
	Vector3 axisA;
	float maxGap;
	int faceDimA = 0, edgeDimA = 0;

	// face axes

	// can compute all the gaps at once with VU0

	Vector3 gapsA = absPerElem(offsetAB) - boxA.half - absCapsDirection * capsuleB.hLength;

	AaxisTest( 0, X, true );
	AaxisTest( 1, Y, false );
	AaxisTest( 2, Z, false );

	// cross product axes

	// compute gaps on all cross product axes using some VU0 math.  suppose there's a tradeoff
	// between doing this with SIMD all at once or without SIMD in each cross product test, since
	// some test might exit early.

	Vector3 lsqrs, projOffset, projAhalf;

	Matrix3 crossProdMat = crossMatrix(capsDirection) * identity;
	Matrix3 crossProdMatT = crossMatrix(-capsDirection) * identity;

	lsqrs = mulPerElem( crossProdMatT.getCol0(), crossProdMatT.getCol0() ) +
			mulPerElem( crossProdMatT.getCol1(), crossProdMatT.getCol1() ) +
			mulPerElem( crossProdMatT.getCol2(), crossProdMatT.getCol2() );

	projOffset = crossProdMatT * offsetAB;
	projAhalf = absPerElem(crossProdMatT) * boxA.half;

	Vector3 gapsAxB = absPerElem(projOffset) - projAhalf;

	CrossAxisTest( 0, X );
	CrossAxisTest( 1, Y );
	CrossAxisTest( 2, Z );

	// make axis point from box center towards capsule center.

	if ( dot(axisA,offsetAB) < 0.0f )
		axisA = -axisA;

	// find the face on box whose normal best matches the separating axis. will use the entire
	// face only in degenerate cases.
	//
	// to make things simpler later, change the coordinate system so that the face normal is the z
	// direction.  if an edge cross product axis was chosen above, also align the box edge to the y
	// axis.  this saves the later tests from having to know which face was chosen.  changing the
	// coordinate system involves permuting vector elements, so construct a permutation matrix.
	// I believe this is a faster way to permute a bunch of vectors than using arrays.

	int dimA[3];

	if ( axisType == CROSS_AXIS ) {
		Vector3 absAxisA = Vector3(absPerElem(axisA));

		dimA[1] = edgeDimA;

		if ( edgeDimA == 0 ) {
			if ( absAxisA[1] > absAxisA[2] ) {
				dimA[0] = 2;
				dimA[2] = 1;
			} else                             {
				dimA[0] = 1;
				dimA[2] = 2;
			}
		} else if ( edgeDimA == 1 ) {
			if ( absAxisA[2] > absAxisA[0] ) {
				dimA[0] = 0;
				dimA[2] = 2;
			} else                             {
				dimA[0] = 2;
				dimA[2] = 0;
			}
		} else {
			if ( absAxisA[0] > absAxisA[1] ) {
				dimA[0] = 1;
				dimA[2] = 0;
			} else                             {
				dimA[0] = 0;
				dimA[2] = 1;
			}
		}
	} else {
		dimA[2] = faceDimA;
		dimA[0] = (faceDimA+1)%3;
		dimA[1] = (faceDimA+2)%3;
	}

	Matrix3 aperm_col;

	aperm_col.setCol0(ident[dimA[0]]);
	aperm_col.setCol1(ident[dimA[1]]);
	aperm_col.setCol2(ident[dimA[2]]);

	Matrix3 aperm_row = transpose(aperm_col);

	// permute vectors to be in face coordinate system.

	Vector3 offsetAB_perm = aperm_row * offsetAB;
	Vector3 halfA_perm = aperm_row * boxA.half;
	Vector3 signsA_perm = copySignPerElem(Vector3(1.0f), aperm_row * axisA);
	Vector3 scalesA_perm = mulPerElem( signsA_perm, halfA_perm );
	Vector3 capsDirection_perm = aperm_row * capsDirection;
	float signB = (-dot(capsDirection,axisA) > 0.0f)? 1.0f : -1.0f;
	float scaleB = signB * capsuleB.hLength;

	// compute the vector between the center of the box face and the capsule center

	offsetAB_perm.setZ( offsetAB_perm.getZ() - scalesA_perm.getZ() );

	// if box and capsule overlap, this will separate them for finding points of penetration.

	if ( maxGap < 0.0f ) {
		offsetAB_perm -= aperm_row * axisA * maxGap * 1.01f;
	}

	// for each vertex/face or edge/edge pair of box face and line segment, find the closest
	// points.
	//
	// these points each have an associated feature (vertex, edge, or face).  if each
	// point is in the external Voronoi region of the other's feature, they are the
	// closest points of the objects, and the algorithm can exit.
	//
	// the feature pairs are arranged so that in the general case, the first test will
	// succeed.  degenerate cases (line segment parallel to face) may require up to all tests
	// in the worst case.
	//
	// if for some reason no case passes the Voronoi test, the features with the minimum
	// distance are returned.

	Vector3 closestPtsVec_perm;
	Point3 localPointA_perm;
	float minDistSqr;
	float segmentParamB;
	bool done;

	localPointA_perm.setZ( scalesA_perm.getZ() );
	scalesA_perm.setZ(0.0f);

	Vector3 hA_perm( halfA_perm );

	int otherFaceDimA;
	FeatureType featureA, featureB;

	if ( axisType == CROSS_AXIS ) {
		EdgeEdgeTests( done, minDistSqr, closestPtsVec_perm, localPointA_perm, segmentParamB,
					   otherFaceDimA, featureA, featureB,
					   hA_perm, capsuleB.hLength, offsetAB_perm, capsDirection_perm, signsA_perm,
					   scalesA_perm, true );

		if ( !done ) {
			VertexBFaceATests( done, minDistSqr, closestPtsVec_perm, localPointA_perm, segmentParamB,
							   featureA, featureB,
							   hA_perm, offsetAB_perm, capsDirection_perm, signB, scaleB, false );
		}
	} else {
		VertexBFaceATests( done, minDistSqr, closestPtsVec_perm, localPointA_perm, segmentParamB,
						   featureA, featureB,
						   hA_perm, offsetAB_perm, capsDirection_perm, signB, scaleB, true );

		if ( !done ) {
			EdgeEdgeTests( done, minDistSqr, closestPtsVec_perm, localPointA_perm, segmentParamB,
						   otherFaceDimA, featureA, featureB,
						   hA_perm, capsuleB.hLength, offsetAB_perm, capsDirection_perm, signsA_perm,
						   scalesA_perm, false );
		}
	}

	// compute normal

	bool centerInside = ( signsA_perm.getZ() * closestPtsVec_perm.getZ() < 0.0f );

	if ( centerInside || ( minDistSqr < lenSqrTol ) ) {
		normal = matrixA * axisA;
	} else {
		Vector3 closestPtsVec = aperm_col * closestPtsVec_perm;
		normal = matrixA * ( closestPtsVec * (1.0f/sqrtf( minDistSqr )) );
	}

	// compute box point

	boxPointA.localPoint = Point3( aperm_col * Vector3( localPointA_perm ) );

	// compute capsule point

	capsulePointB.lineParam = segmentParamB;
	capsulePointB.localPoint = Point3( transpose(transformB.getUpper3x3()) * ( directionB * segmentParamB - normal * capsuleB.radius ) );

	int sA[3];

	sA[0] = boxPointA.localPoint.getX() > 0.0f;
	sA[1] = boxPointA.localPoint.getY() > 0.0f;
	sA[2] = boxPointA.localPoint.getZ() > 0.0f;

	if ( featureA == F ) {
		boxPointA.setFaceFeature( dimA[2], sA[dimA[2]] );
	} else if ( featureA == E ) {
		boxPointA.setEdgeFeature( dimA[2], sA[dimA[2]], dimA[otherFaceDimA], sA[dimA[otherFaceDimA]] );
	} else {
		boxPointA.setVertexFeature( sA[0], sA[1], sA[2] );
	}

	if ( featureB == E ) {
		capsulePointB.setEdgeFeature();
	} else if ( featureB == V ) {
		capsulePointB.setVertexFeature( capsulePointB.lineParam > 0.0f );
	}

	// return distance

	if ( centerInside ) {
		return (-sqrtf( minDistSqr ) - capsuleB.radius);
	} else {
		return (sqrtf( minDistSqr ) - capsuleB.radius);
	}
}

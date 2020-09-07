//
//  KGCommon.h
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#ifndef __KG_Common_H__
#define __KG_Common_H__

#include <cstdint>
#include <vector>
#include <limits>

namespace KG
{
	class Vector3;
	class Vector2
	{
	public:
		double x;
		double y;
		
		double GetDotProduct(const Vector2 &other) const;
		float GetCrossProduct(const Vector2 &other) const;
		Vector3 GetCrossProductSimplified(const Vector2 &other) const;
		double GetDotProductSimplified(const Vector3 &other) const;
	};

	class Vector3
	{
	public:
		double x;
		double y;
		double z;
		
		Vector3 GetCrossProduct(const Vector3 &other) const;
		double GetDotProduct(const Vector3 &other) const;
		double GetDotProductSimplified(const Vector2 &other) const;
	};

	class PathSegment
	{
	public:
		enum Type
		{
			TypePoint,
			TypeLine,
			TypeBezierQuadratic,
			TypeBezierCubic,
			TypeArc
		};
		
		Type type;
		std::vector<Vector2> controlPoints;
	};

	class Path
	{
	public:
		std::vector<PathSegment> segments;
	};

	class PathCollection
	{
	public:
		std::vector<Path> paths;
	};

	class TriangleMesh
	{
	public:
		enum VertexFeature
		{
			VertexFeaturePosition,
			VertexFeatureUV,
			VertexFeatureColor
		};
		
		std::vector<VertexFeature> features;
		std::vector<float> vertices;
		std::vector<uint32_t> indices;
	};

	class Math
	{
	public:
		static bool IsCCW(const Vector2 &A, const Vector2 &B, const Vector2 &C);
		static int8_t IsOnLine(const Vector2 &A, const Vector2 &B, const Vector2 &C, double epsilon = std::numeric_limits<double>::epsilon());
		static bool AreLineSegmentsIntersecting(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D);
		static bool AreTrianglesIntersecting(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E, const Vector2 &F);
		static float GetSquaredTriangleArea(const Vector2 &A, const Vector2 &B, const Vector2 &C);
		static Vector2 GetIntersectionPoint(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D);
		static bool IsQuadraticCurveIntersectingLineSegment(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E);
		static std::vector<double> GetQuadraticCurveAndLineSegmentIntersectionCoefficients(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E);
		static std::vector<double> GetQuadraticCurveAndQuadraticCurveIntersectionCoefficients(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E, const Vector2 &F);
	};
}

#endif /* defined(__KG_Common_H__) */

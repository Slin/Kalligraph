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

namespace KG
{
	class Vector3;
	class Vector2
	{
	public:
		double x;
		double y;
		
		double GetDotProduct(const Vector2 &other) const;
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
}

#endif /* defined(__KG_Common_H__) */

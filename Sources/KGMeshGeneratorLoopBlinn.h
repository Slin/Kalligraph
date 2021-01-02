//
//  KGMeshGeneratorLoopBlinn.h
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#ifndef __KG_MeshGeneratorLoopBlinn_H__
#define __KG_MeshGeneratorLoopBlinn_H__

#include "KGCommon.h"
#include "KGTriangulatorBruteForce.h"

namespace KG
{
	class MeshGeneratorLoopBlinn
	{
	public:
		static const PathCollection DowngradeCubicSegments(const PathCollection &paths);
		static const PathCollection FilterDegenerateSegments(const PathCollection &paths, double epsilon = std::numeric_limits<double>::epsilon());
		static const PathCollection FindWindingOrder(const PathCollection &paths);
		static const PathCollection ResolveIntersections(const PathCollection &paths);
		static const PathCollection ResolveOverlaps(const PathCollection &paths, double minTriangleArea = 0.001);
		
		static const TriangleMesh GetMeshForPathCollection(const PathCollection &paths);
		
	private:
		static PathSegment GetQuadraticSegmentForCubic(const PathSegment &segment);
		static void ResolveLineLineIntersection(std::vector<PathSegment> &iteratedPathSegments, std::vector<PathSegment> &otherPathSegments);
		static void ResolveQuadraticLineIntersection(std::vector<PathSegment> &iteratedPathSegments, std::vector<PathSegment> &otherPathSegments);
		static void ResolveQuadraticQuadraticIntersection(std::vector<PathSegment> &iteratedPathSegments, std::vector<PathSegment> &otherPathSegments);
		static void ResolveQuadraticQuadraticOverlap(std::vector<PathSegment> &iteratedPathSegments, std::vector<PathSegment> &otherPathSegments, double minTriangleArea);
	};
}

#endif /* defined(__KG_MeshGeneratorLoopBlinn_H__) */

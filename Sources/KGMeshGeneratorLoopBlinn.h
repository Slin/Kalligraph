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
		static const PathCollection ResolveIntersections(const PathCollection &paths);
		static const PathCollection ResolveOverlaps(const PathCollection &paths);
		
		static const TriangleMesh GetMeshForPathCollection(const PathCollection &paths, bool isCCW);
		
	private:
		static PathSegment GetQuadraticSegmentForCubic(const PathSegment &segment);
		static void ResolveQuadraticQuadraticOverlap(std::vector<PathSegment> &oldPathSegments, std::vector<PathSegment> &newPathSegments);
	};
}

#endif /* defined(__KG_MeshGeneratorLoopBlinn_H__) */

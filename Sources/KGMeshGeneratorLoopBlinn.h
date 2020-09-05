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
		static const TriangleMesh GetMeshForPathCollection(const PathCollection &paths, bool isCCW);
		
	private:
		static PathSegment GetQuadraticSegmentForCubic(const PathSegment &segment);
		static const PathCollection DowngradeCubicSegments(const PathCollection &paths);
		static const PathCollection FilterDegenerateSegments(const PathCollection &paths);
		static const PathCollection ResolveOverlaps(const PathCollection &paths);
	};
}

#endif /* defined(__KG_MeshGeneratorLoopBlinn_H__) */

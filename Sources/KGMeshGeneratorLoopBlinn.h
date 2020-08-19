//
//  KGMeshGeneratorLoopBlinn.h
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#ifndef __KG_MeshGeneratorLoopBlinn_H__
#define __KG_MeshGeneratorLoopBlinn_H__

#include "KGCommon.h"
#include "KGBruteForceTriangulator.h"

namespace KG
{
	class MeshGeneratorLoopBlinn
	{
	public:
		static const TriangleMesh GetMeshForPathCollection(const PathCollection &paths, bool isCCW);
		
	private:
		static void AddLineSegmentToOutline(BruteForceTriangulator::Outline &outline, const PathSegment &segment);
		static void AddQuadraticSegmentToOutline(BruteForceTriangulator::Outline &outline, TriangleMesh &outsideMesh, const PathSegment &segment, const bool &isCCW);
		static void AddCubicSegmentToOutline(BruteForceTriangulator::Outline &outline, TriangleMesh &outsideMesh, const PathSegment &segment, const bool &isCCW);
	};
}

#endif /* defined(__KG_MeshGeneratorLoopBlinn_H__) */

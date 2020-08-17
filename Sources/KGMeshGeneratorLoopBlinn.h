//
//  KGMeshGeneratorLoopBlinn.h
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#ifndef __KG_MeshGeneratorLoopBlinn_H__
#define __KG_MeshGeneratorLoopBlinn_H__

#include "KGCommon.h"

namespace KG
{
	class MeshGeneratorLoopBlinn
	{
	public:
		static const TriangleMesh GetMeshForPathCollection(const PathCollection &paths, bool isCCW);
	};
}

#endif /* defined(__KG_MeshGeneratorLoopBlinn_H__) */

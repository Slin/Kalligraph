//
//  KGMeshGeneratorLoopBlinn.cpp
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#include "KGMeshGeneratorLoopBlinn.h"

#include "KGBruteForceTriangulator.h"
#include <cmath>

namespace KG
{
	const TriangleMesh MeshGeneratorLoopBlinn::GetMeshForPathCollection(const PathCollection &paths)
	{
/*		for(const Path &path : paths.paths)
		{
			for(const PathSegment &segment : path.segments)
			{
				int32_t startIndex = mesh.vertices.size();
				
				switch(segment.type)
				{
					case PathSegment::TypeBezierCubic:
					{
						const Vector2 &b0 = segment.controlPoints[0];
						const Vector2 &b1 = segment.controlPoints[1];
						const Vector2 &b2 = segment.controlPoints[2];
						const Vector2 &b3 = segment.controlPoints[3];
						
						float a1 = b0.GetDotProductSimplified(b3.GetCrossProductSimplified(b2));
						float a2 = b1.GetDotProductSimplified(b0.GetCrossProductSimplified(b3));
						float a3 = b2.GetDotProductSimplified(b1.GetCrossProductSimplified(b0));

						float d1 = a1 - 2 * a2 + 3 * a3;
						float d2 = -a2 + 3 * a3;
						float d3 = 3 * a3;
						
						float discr = d1 * d1 * (3 * d2 * d2 - 4 * d1 * d3);
						
						if(discr > 0) //Serpentine
						{
							float root = std::sqrt(9.0f * d2 * d2 - 12.0f * d1 * d3);

							float ls = 3 * d2 - root;
							float lt = 6 * d1;

							float ms = 3 * d2 + root;
							float mt = 6 * d1;

							float kmn[12];
							
							kmn[0] = ls * ms;
							kmn[1] = ls * ls * ls;
							kmn[2] = ms * ms * ms;
							
							kmn[3] = (3.0f*ls*ms - ls*mt - lt*ms)/3.0f;
							kmn[4] = ls*ls*(ls-lt);
							kmn[5] = ms*ms*(ms-mt);
							
							kmn[6] = (lt*(mt-2*ms) + ls*(3*ms - 2*mt))/3;
							kmn[7] = (lt-ls)*(lt-ls)*ls;
							kmn[8] = (mt-ms)*(mt-ms)*ms;
							
							kmn[9] = (lt-ls)*(mt-ms);
							kmn[10] = -(lt-ls)*(lt-ls)*(lt-ls);
							kmn[11] = -(mt-ms)*(mt-ms)*(mt-ms);
							
							if(d1 >= 0.0f)
							{
								mesh.vertices.push_back(b0.x);
								mesh.vertices.push_back(b0.y);
								
								mesh.vertices.push_back(kmn[0]);
								mesh.vertices.push_back(kmn[1]);
								mesh.vertices.push_back(kmn[2]);
								
								mesh.vertices.push_back(b1.x);
								mesh.vertices.push_back(b1.y);
								
								mesh.vertices.push_back(kmn[3]);
								mesh.vertices.push_back(kmn[4]);
								mesh.vertices.push_back(kmn[5]);
								
								mesh.vertices.push_back(b2.x);
								mesh.vertices.push_back(b2.y);
								
								mesh.vertices.push_back(kmn[6]);
								mesh.vertices.push_back(kmn[7]);
								mesh.vertices.push_back(kmn[8]);
								
								mesh.vertices.push_back(b3.x);
								mesh.vertices.push_back(b3.y);
								
								mesh.vertices.push_back(kmn[9]);
								mesh.vertices.push_back(kmn[10]);
								mesh.vertices.push_back(kmn[11]);
							}
							else
							{
								mesh.vertices.push_back(b0.x);
								mesh.vertices.push_back(b0.y);
								
								mesh.vertices.push_back(-kmn[0]);
								mesh.vertices.push_back(-kmn[1]);
								mesh.vertices.push_back(-kmn[2]);
								
								mesh.vertices.push_back(b1.x);
								mesh.vertices.push_back(b1.y);
								
								mesh.vertices.push_back(-kmn[3]);
								mesh.vertices.push_back(-kmn[4]);
								mesh.vertices.push_back(-kmn[5]);
								
								mesh.vertices.push_back(b2.x);
								mesh.vertices.push_back(b2.y);
								
								mesh.vertices.push_back(-kmn[6]);
								mesh.vertices.push_back(-kmn[7]);
								mesh.vertices.push_back(-kmn[8]);
								
								mesh.vertices.push_back(b3.x);
								mesh.vertices.push_back(b3.y);
								
								mesh.vertices.push_back(-kmn[9]);
								mesh.vertices.push_back(-kmn[10]);
								mesh.vertices.push_back(-kmn[11]);
							}
							
							mesh.indices.push_back(startIndex + 0);
							mesh.indices.push_back(startIndex + 1);
							mesh.indices.push_back(startIndex + 2);
							
							mesh.indices.push_back(startIndex + 1);
							mesh.indices.push_back(startIndex + 3);
							mesh.indices.push_back(startIndex + 2);
						}
						else if(discr < 0) // Loop
						{
							
						}
						else // Cusp
						{
							
						}
						
						break;
					}
				}
			}
		}*/

		
		BruteForceTriangulator::Polygon polygon;
		
		for(const Path &path : paths.paths)
		{
			BruteForceTriangulator::Outline outline;
			
			bool isNotFirstSegment = false;
			for(const PathSegment &segment : path.segments)
			{
				for(const Vector2 point : segment.controlPoints)
				{
					if(isNotFirstSegment)
					{
						isNotFirstSegment = true;
						continue;
					}
					
					outline.points.push_back(point);
				}
			}
			
			polygon.outlines.push_back(outline);
		}
		
		return BruteForceTriangulator::Triangulate(polygon);
	}
}

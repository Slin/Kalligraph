//
//  KGMeshGeneratorLoopBlinn.cpp
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#include "KGMeshGeneratorLoopBlinn.h"

#include "KGBruteForceTriangulator.h"
#include <cmath>
#include <limits>

namespace KG
{
	int8_t onLineOther(const Vector2 &A, const Vector2 &B, const Vector2 &C)
	{
		double result = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
		if(std::abs(result) < std::numeric_limits<double>::epsilon()) return 0;
		if(result < 0) return -1;
		return 1;
	}

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

		
		uint32_t vertexIndex = 0;
		TriangleMesh outsideMesh;
		outsideMesh.features.push_back(TriangleMesh::VertexFeaturePosition);
		outsideMesh.features.push_back(TriangleMesh::VertexFeatureUV);
		
		BruteForceTriangulator::Polygon polygon;
		
		for(const Path &path : paths.paths)
		{
			BruteForceTriangulator::Outline outline;
			
			bool isNotFirstSegment = false;
			for(const PathSegment &segment : path.segments)
			{
				if(segment.type == PathSegment::TypeBezierQuadratic)
				{
					Vector2 BA;
					BA.x = segment.controlPoints[2].x - segment.controlPoints[0].x;
					BA.y = segment.controlPoints[2].y - segment.controlPoints[0].y;
					
					Vector2 CA;
					CA.x = segment.controlPoints[1].x - segment.controlPoints[0].x;
					CA.y = segment.controlPoints[1].y - segment.controlPoints[0].y;
					
					Vector2 CB;
					CB.x = segment.controlPoints[1].x - segment.controlPoints[2].x;
					CB.y = segment.controlPoints[1].y - segment.controlPoints[2].y;
					
					bool needsStartPoint = !isNotFirstSegment;
					bool needsControlPoint = true;
					bool needsEndPoint = true;
					if(BA.GetDotProduct(BA) < std::numeric_limits<double>::epsilon() || CB.GetDotProduct(CB) < std::numeric_limits<double>::epsilon())
					{
						needsControlPoint = false;
					}
					
					if(CA.GetDotProduct(CA) < std::numeric_limits<double>::epsilon())
					{
						needsEndPoint = false;
						needsControlPoint = false;
					}
					
					if(needsControlPoint)
					{
						int8_t onLineResult = onLineOther(segment.controlPoints[0], segment.controlPoints[2], segment.controlPoints[1]);
						if(onLineResult == 0) needsControlPoint = false;
						else if(onLineResult < 0)
						{
							//Is outside curve
							needsControlPoint = false;
							outsideMesh.vertices.push_back(segment.controlPoints[0].x);
							outsideMesh.vertices.push_back(segment.controlPoints[0].y);
							outsideMesh.vertices.push_back(0.0f);
							outsideMesh.vertices.push_back(0.0f);
							outsideMesh.vertices.push_back(1.0f);
							
							outsideMesh.vertices.push_back(segment.controlPoints[1].x);
							outsideMesh.vertices.push_back(segment.controlPoints[1].y);
							outsideMesh.vertices.push_back(0.5f);
							outsideMesh.vertices.push_back(0.0f);
							outsideMesh.vertices.push_back(1.0f);
							
							outsideMesh.vertices.push_back(segment.controlPoints[2].x);
							outsideMesh.vertices.push_back(segment.controlPoints[2].y);
							outsideMesh.vertices.push_back(1.0f);
							outsideMesh.vertices.push_back(1.0f);
							outsideMesh.vertices.push_back(1.0f);
							
							outsideMesh.indices.push_back(vertexIndex++);
							outsideMesh.indices.push_back(vertexIndex++);
							outsideMesh.indices.push_back(vertexIndex++);
						}
						else
						{
							//Is inside curve
							outsideMesh.vertices.push_back(segment.controlPoints[0].x);
							outsideMesh.vertices.push_back(segment.controlPoints[0].y);
							outsideMesh.vertices.push_back(0.0f);
							outsideMesh.vertices.push_back(0.0f);
							outsideMesh.vertices.push_back(-1.0f);
							
							outsideMesh.vertices.push_back(segment.controlPoints[1].x);
							outsideMesh.vertices.push_back(segment.controlPoints[1].y);
							outsideMesh.vertices.push_back(0.5f);
							outsideMesh.vertices.push_back(0.0f);
							outsideMesh.vertices.push_back(-1.0f);
							
							outsideMesh.vertices.push_back(segment.controlPoints[2].x);
							outsideMesh.vertices.push_back(segment.controlPoints[2].y);
							outsideMesh.vertices.push_back(1.0f);
							outsideMesh.vertices.push_back(1.0f);
							outsideMesh.vertices.push_back(-1.0f);
							
							outsideMesh.indices.push_back(vertexIndex++);
							outsideMesh.indices.push_back(vertexIndex++);
							outsideMesh.indices.push_back(vertexIndex++);
						}
					}
						
					if(isNotFirstSegment)
					{
						isNotFirstSegment = true;
					}
					
					if(needsStartPoint)
						outline.points.push_back(segment.controlPoints[0]);
					if(needsControlPoint)
						outline.points.push_back(segment.controlPoints[1]);
					if(needsEndPoint)
						outline.points.push_back(segment.controlPoints[2]);
				}
			}
			
			polygon.outlines.push_back(outline);
		}
		
		TriangleMesh insideMesh = BruteForceTriangulator::Triangulate(polygon);
		uint32_t vertexIndexOffset = insideMesh.vertices.size() / 5;
		insideMesh.vertices.insert(insideMesh.vertices.end(), outsideMesh.vertices.begin(), outsideMesh.vertices.end());
		for(uint32_t index : outsideMesh.indices)
		{
			insideMesh.indices.push_back(index + vertexIndexOffset);
		}
		
		return insideMesh;
	}
}

//
//  KGMeshGeneratorLoopBlinn.cpp
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#include "KGMeshGeneratorLoopBlinn.h"

#include <cmath>
#include <limits>

namespace KG
{
	//Turns a cubic segment into a quadratic one
	PathSegment MeshGeneratorLoopBlinn::GetQuadraticSegmentForCubic(const PathSegment &segment)
	{
		//Turn cubic into quadratic
		Vector2 control;
		control.x = -0.25f * segment.controlPoints[0].x + 0.75f * segment.controlPoints[1].x + 0.75f * segment.controlPoints[2].x - 0.25f * segment.controlPoints[3].x;
		control.y = -0.25f * segment.controlPoints[0].y + 0.75f * segment.controlPoints[1].y + 0.75f * segment.controlPoints[2].y - 0.25f * segment.controlPoints[3].y;
		
		PathSegment quadraticSegment;
		quadraticSegment.type = PathSegment::TypeBezierQuadratic;
		quadraticSegment.controlPoints.push_back(segment.controlPoints[0]);
		quadraticSegment.controlPoints.push_back(control);
		quadraticSegment.controlPoints.push_back(segment.controlPoints[3]);
		
		return quadraticSegment;
	}

	//Converts all cubic segments into quadratic ones.
	//Depending on the segments this may introduce a lot of error (subdividing before the conversion would help reduce the error).
	const PathCollection MeshGeneratorLoopBlinn::DowngradeCubicSegments(const PathCollection &paths)
	{
		PathCollection result;
		
		for(const Path &path : paths.paths)
		{
			result.paths.push_back(Path());
			for(const PathSegment &segment : path.segments)
			{
				if(segment.type == PathSegment::TypeBezierCubic)
				{
					result.paths.back().segments.push_back(GetQuadraticSegmentForCubic(segment));
				}
				else
				{
					result.paths.back().segments.push_back(segment);
				}
			}
			
			//If new path is empty, remove it
			if(result.paths.back().segments.size() == 0)
			{
				result.paths.pop_back();
			}
		}
		
		return result;
	}

	//Reduces higher order segments to lower order with epsilon as threshold.
	//This will simplify the generated mesh if the input data used a higher order representation for lower order segments (nanosvg turns everything into cubic curves)
	//Points will be discarded
	const PathCollection MeshGeneratorLoopBlinn::FilterDegenerateSegments(const PathCollection &paths, double epsilon)
	{
		PathCollection result;
		
		for(const Path &path : paths.paths)
		{
			result.paths.push_back(Path());
			for(const PathSegment &segment : path.segments)
			{
				PathSegment newSegment = segment;
				if(newSegment.type == PathSegment::TypeBezierCubic)
				{
					//Cubics are currently not supported here and should have been converted in a previous step already.
				}
				else if(newSegment.type == PathSegment::TypeBezierQuadratic)
				{
					Vector2 BA;
					BA.x = newSegment.controlPoints[2].x - newSegment.controlPoints[0].x;
					BA.y = newSegment.controlPoints[2].y - newSegment.controlPoints[0].y;
					
					Vector2 CA;
					CA.x = newSegment.controlPoints[1].x - newSegment.controlPoints[0].x;
					CA.y = newSegment.controlPoints[1].y - newSegment.controlPoints[0].y;
					
					Vector2 CB;
					CB.x = newSegment.controlPoints[1].x - newSegment.controlPoints[2].x;
					CB.y = newSegment.controlPoints[1].y - newSegment.controlPoints[2].y;
					
					bool needsControlPoint = true;
					bool needsEndPoint = true;
					if(BA.GetDotProduct(BA) < epsilon || CB.GetDotProduct(CB) < epsilon)
					{
						needsControlPoint = false;
					}
					
					if(CA.GetDotProduct(CA) < epsilon)
					{
						needsEndPoint = false;
						needsControlPoint = false;
					}
					
					if(needsControlPoint)
					{
						int8_t onLineResult = Math::IsOnLine(newSegment.controlPoints[0], newSegment.controlPoints[2], newSegment.controlPoints[1], epsilon);
						if(onLineResult == 0) needsControlPoint = false;
					}
					
					if(!needsControlPoint)
					{
						newSegment.type = PathSegment::TypeLine;
						newSegment.controlPoints.erase(newSegment.controlPoints.begin() + 1);
					}
					if(!needsEndPoint)
					{
						newSegment.type = PathSegment::TypePoint;
					}
				}
				else if(newSegment.type == PathSegment::TypeLine)
				{
					Vector2 BA;
					BA.x = newSegment.controlPoints[1].x - newSegment.controlPoints[0].x;
					BA.y = newSegment.controlPoints[1].y - newSegment.controlPoints[0].y;
					
					//If both points are identical, turn this into a point
					if(BA.GetDotProduct(BA) <= epsilon)
					{
						newSegment.type = PathSegment::TypePoint;
					}
				}
				
				//Only add segment if NOT a point
				if(newSegment.type != PathSegment::TypePoint)
				{
					result.paths.back().segments.push_back(newSegment);
				}
			}
			
			//If new path is empty, remove it
			if(result.paths.back().segments.size() == 0)
			{
				result.paths.pop_back();
			}
		}
		
		return result;
	}


	//Checks if two line segments intersect and splits them if they do.
	void MeshGeneratorLoopBlinn::ResolveLineLineIntersection(std::vector<PathSegment> &iteratedPathSegments, std::vector<PathSegment> &otherPathSegments)
	{
		const PathSegment otherSegment = otherPathSegments.back();
		for(int i = 0; i < iteratedPathSegments.size(); i++)
		{
			const PathSegment iteratedSegment = iteratedPathSegments[i];
			
			//Only intersect if they don't share a control point (these are all doubles, but an epsilon for the comparison should not be needed as they should be identical if shared)
			if(otherSegment.controlPoints[0].x != iteratedSegment.controlPoints[0].x && otherSegment.controlPoints[0].x != iteratedSegment.controlPoints[1].x && otherSegment.controlPoints[1].x != iteratedSegment.controlPoints[0].x && otherSegment.controlPoints[1].x != iteratedSegment.controlPoints[1].x && otherSegment.controlPoints[0].y != iteratedSegment.controlPoints[0].y && otherSegment.controlPoints[0].y != iteratedSegment.controlPoints[1].y && otherSegment.controlPoints[1].y != iteratedSegment.controlPoints[0].y && otherSegment.controlPoints[1].y != iteratedSegment.controlPoints[1].y && Math::AreLineSegmentsIntersecting(otherSegment.controlPoints[0], otherSegment.controlPoints[1], iteratedSegment.controlPoints[0], iteratedSegment.controlPoints[1]))
			{
				otherPathSegments.pop_back(); //Remove the segment that gets split
				iteratedPathSegments.erase(iteratedPathSegments.begin() + i); //Remove the segment that gets split
				
				Vector2 intersectionPoint = Math::GetIntersectionPoint(otherSegment.controlPoints[0], otherSegment.controlPoints[1], iteratedSegment.controlPoints[0], iteratedSegment.controlPoints[1]);
				
				PathSegment subdividedSegment[2];
				subdividedSegment[0].type = PathSegment::TypeLine;
				subdividedSegment[1].type = PathSegment::TypeLine;
				
				subdividedSegment[0].controlPoints.push_back(otherSegment.controlPoints[0]);
				subdividedSegment[0].controlPoints.push_back(intersectionPoint);
				otherPathSegments.push_back(subdividedSegment[0]);
				
				subdividedSegment[1].controlPoints.push_back(intersectionPoint);
				subdividedSegment[1].controlPoints.push_back(otherSegment.controlPoints[1]);
				otherPathSegments.push_back(subdividedSegment[1]);
				
				subdividedSegment[0].controlPoints[0] = iteratedSegment.controlPoints[0];
				subdividedSegment[0].controlPoints[1] = intersectionPoint;
				iteratedPathSegments.insert(iteratedPathSegments.begin() + i, subdividedSegment[0]);
				i += 1;
				
				subdividedSegment[1].controlPoints[0] = intersectionPoint;
				subdividedSegment[1].controlPoints[1] = iteratedSegment.controlPoints[1];
				iteratedPathSegments.insert(iteratedPathSegments.begin() + i, subdividedSegment[1]);
			}
		}
	}


	//Splits intersecting segments at the intersection point.
	//ResolveOverlaps will get stuck on intersections otherwise and the resulting mesh will be broken if this isn't handled.
	const PathCollection MeshGeneratorLoopBlinn::ResolveIntersections(const PathCollection &paths)
	{
		PathCollection result;
		
		for(const Path &path : paths.paths)
		{
			result.paths.push_back(Path());
			for(const PathSegment &segment : path.segments)
			{
				std::vector<PathSegment> newSegments;
				newSegments.push_back(segment);
				
				//Check if the new segment intersects any of the ones that have already been added
				for(Path &otherPath : result.paths)
				{
					for(int i = 0; i < otherPath.segments.size(); i++)
					{
						//Handle Line - Line segment intersection
						if(segment.type == PathSegment::TypeLine && otherPath.segments[i].type == PathSegment::TypeLine)
						{
							std::vector<PathSegment> oldSegments;
							oldSegments.push_back(otherPath.segments[i]);
							ResolveLineLineIntersection(newSegments, oldSegments);
							
							//Update the old segment if it was split
							if(oldSegments.size() > 1)
							{
								otherPath.segments.erase(otherPath.segments.begin() + i);
								otherPath.segments.insert(otherPath.segments.begin() + i, oldSegments.begin(), oldSegments.end());
								i += oldSegments.size() - 1;
							}
						}
					}
				}
				
				result.paths.back().segments.insert(result.paths.back().segments.end(), newSegments.begin(), newSegments.end());
			}
			
			//If new path is empty, remove it
			if(result.paths.back().segments.size() == 0)
			{
				result.paths.pop_back();
			}
		}
		
		return result;
	}

	//Checks if two quadratic segment triangles overlap (but the curves are not allowed to intersect!) and subdivides them until they don't.
	void MeshGeneratorLoopBlinn::ResolveQuadraticQuadraticOverlap(std::vector<PathSegment> &iteratedPathSegments, std::vector<PathSegment> &otherPathSegments)
	{
		const PathSegment otherSegment = otherPathSegments.back();
		for(int i = 0; i < iteratedPathSegments.size(); i++)
		{
			const PathSegment iteratedSegment = iteratedPathSegments[i];
			
			if(Math::AreTrianglesIntersecting(otherSegment.controlPoints[0], otherSegment.controlPoints[1], otherSegment.controlPoints[2], iteratedSegment.controlPoints[0], iteratedSegment.controlPoints[1], iteratedSegment.controlPoints[2]))
			{
				if(Math::GetSquaredTriangleArea(otherSegment.controlPoints[0], otherSegment.controlPoints[1], otherSegment.controlPoints[2]) > Math::GetSquaredTriangleArea(iteratedSegment.controlPoints[0], iteratedSegment.controlPoints[1], iteratedSegment.controlPoints[2]))
				{
					//Subdivide other segment if it's bigger
					
					otherPathSegments.pop_back(); //Remove the triangle that gets subdivided
					PathSegment subdividedSegment[2];
					
					subdividedSegment[0].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[0].controlPoints.push_back(otherSegment.controlPoints[0]);
					subdividedSegment[0].controlPoints.push_back({0.5f * (otherSegment.controlPoints[0].x + otherSegment.controlPoints[1].x), 0.5f * (otherSegment.controlPoints[0].y + otherSegment.controlPoints[1].y)});
					subdividedSegment[0].controlPoints.push_back({0.25f * (otherSegment.controlPoints[0].x + otherSegment.controlPoints[2].x) + 0.5 * otherSegment.controlPoints[1].x, 0.25f * (otherSegment.controlPoints[0].y + otherSegment.controlPoints[2].y) + 0.5 * otherSegment.controlPoints[1].y});
					
					otherPathSegments.push_back(subdividedSegment[0]);
					ResolveQuadraticQuadraticOverlap(iteratedPathSegments, otherPathSegments);
					
					subdividedSegment[1].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[1].controlPoints.push_back(subdividedSegment[0].controlPoints[2]);
					subdividedSegment[1].controlPoints.push_back({0.5f * (otherSegment.controlPoints[1].x + otherSegment.controlPoints[2].x), 0.5f * (otherSegment.controlPoints[1].y + otherSegment.controlPoints[2].y)});
					subdividedSegment[1].controlPoints.push_back(otherSegment.controlPoints[2]);
					
					otherPathSegments.push_back(subdividedSegment[1]);
					ResolveQuadraticQuadraticOverlap(iteratedPathSegments, otherPathSegments);
				}
				else
				{
					//Subdivide iterated segment if it's bigger
					
					iteratedPathSegments.erase(iteratedPathSegments.begin() + i); //Remove the segment that gets subdivided
					PathSegment subdividedSegment[2];
					
					subdividedSegment[0].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[0].controlPoints.push_back(iteratedSegment.controlPoints[0]);
					subdividedSegment[0].controlPoints.push_back({0.5f * (iteratedSegment.controlPoints[0].x + iteratedSegment.controlPoints[1].x), 0.5f * (iteratedSegment.controlPoints[0].y + iteratedSegment.controlPoints[1].y)});
					subdividedSegment[0].controlPoints.push_back({0.25f * (iteratedSegment.controlPoints[0].x + iteratedSegment.controlPoints[2].x) + 0.5 * iteratedSegment.controlPoints[1].x, 0.25f * (iteratedSegment.controlPoints[0].y + iteratedSegment.controlPoints[2].y) + 0.5 * iteratedSegment.controlPoints[1].y});
					
					std::vector<PathSegment> newSegments0;
					newSegments0.push_back(subdividedSegment[0]);
					ResolveQuadraticQuadraticOverlap(otherPathSegments, newSegments0);
					iteratedPathSegments.insert(iteratedPathSegments.begin() + i, newSegments0.begin(), newSegments0.end());
					i += newSegments0.size();
					
					subdividedSegment[1].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[1].controlPoints.push_back(subdividedSegment[0].controlPoints[2]);
					subdividedSegment[1].controlPoints.push_back({0.5f * (iteratedSegment.controlPoints[1].x + iteratedSegment.controlPoints[2].x), 0.5f * (iteratedSegment.controlPoints[1].y + iteratedSegment.controlPoints[2].y)});
					subdividedSegment[1].controlPoints.push_back(iteratedSegment.controlPoints[2]);
					
					std::vector<PathSegment> newSegments1;
					newSegments1.push_back(subdividedSegment[1]);
					ResolveQuadraticQuadraticOverlap(otherPathSegments, newSegments1);
					iteratedPathSegments.insert(iteratedPathSegments.begin() + i, newSegments1.begin(), newSegments1.end());
					i += newSegments1.size() - 1;
				}
			}
		}
	}

	const PathCollection MeshGeneratorLoopBlinn::ResolveOverlaps(const PathCollection &paths)
	{
		PathCollection result;
		
		for(const Path &path : paths.paths)
		{
			result.paths.push_back(Path());
			for(const PathSegment &segment : path.segments)
			{
				if(segment.type == PathSegment::TypeBezierCubic)
				{
					//Cubics are currently not supported here and should have been converted in a previous step already.
				}
				else if(segment.type == PathSegment::TypeBezierQuadratic)
				{
					std::vector<PathSegment> newSegments;
					newSegments.push_back(segment);
					
					//Check if the new segment overlaps any of the ones that have already been added
					for(Path &otherPath : result.paths)
					{
						for(int i = 0; i < otherPath.segments.size(); i++)
						{
							//Only subdivide for quadratic with quadratic segment triangle intersection for now
							//Cubic segments are not currently supported, but would need special handling here
							if(otherPath.segments[i].type == PathSegment::TypeBezierQuadratic)
							{
								std::vector<PathSegment> oldSegments;
								oldSegments.push_back(otherPath.segments[i]);
								ResolveQuadraticQuadraticOverlap(newSegments, oldSegments);
								
								//Update the old segment with it's subdivisions if there are any
								if(oldSegments.size() > 1)
								{
									otherPath.segments.erase(otherPath.segments.begin() + i);
									otherPath.segments.insert(otherPath.segments.begin() + i, oldSegments.begin(), oldSegments.end());
									i += oldSegments.size() - 1;
								}
							}
						}
					}
					
					result.paths.back().segments.insert(result.paths.back().segments.end(), newSegments.begin(), newSegments.end());
				}
				else
				{
					result.paths.back().segments.push_back(segment);
				}
			}
			
			//If new path is empty, remove it
			if(result.paths.back().segments.size() == 0)
			{
				result.paths.pop_back();
			}
		}
		
		return result;
	}

	//Turns a path collection into a triangle mesh to render with a quadratic curve shader.
	const TriangleMesh MeshGeneratorLoopBlinn::GetMeshForPathCollection(const PathCollection &paths, bool isCCW)
	{
		PathCollection filteredPaths = DowngradeCubicSegments(paths);
		filteredPaths = FilterDegenerateSegments(filteredPaths, 0.1);
		filteredPaths = ResolveIntersections(filteredPaths);
		filteredPaths = ResolveOverlaps(filteredPaths);
		
		TriangleMesh outsideMesh;
		outsideMesh.features.push_back(TriangleMesh::VertexFeaturePosition);
		outsideMesh.features.push_back(TriangleMesh::VertexFeatureUV);
		
		TriangulatorBruteForce::Polygon polygon;
		
		for(const Path &path : filteredPaths.paths)
		{
			TriangulatorBruteForce::Outline outline;
			
			bool isNotFirstSegment = false;
			for(const PathSegment &segment : path.segments)
			{
				if(!isNotFirstSegment)
				{
					outline.points.push_back(segment.controlPoints[0]);
				}
				isNotFirstSegment = true;
				
				if(segment.type == PathSegment::TypeLine)
				{
					outline.points.push_back(segment.controlPoints[1]);
				}
				else if(segment.type == PathSegment::TypeBezierQuadratic)
				{
					int8_t onLineResult = Math::IsOnLine(segment.controlPoints[0], segment.controlPoints[2], segment.controlPoints[1]);
					int8_t direction = 0;
					if((onLineResult < 0 && isCCW) || (onLineResult > 0 && !isCCW))
					{
						//Is outside curve
						direction = 1;
						
						outline.points.push_back(segment.controlPoints[2]);
					}
					else
					{
						//Is inside curve
						direction = -1;
						
						outline.points.push_back(segment.controlPoints[1]);
						outline.points.push_back(segment.controlPoints[2]);
					}
					
					if(direction != 0)
					{
						outsideMesh.vertices.push_back(segment.controlPoints[0].x);
						outsideMesh.vertices.push_back(segment.controlPoints[0].y);
						outsideMesh.vertices.push_back(0.0f);
						outsideMesh.vertices.push_back(0.0f);
						outsideMesh.vertices.push_back(direction);
						
						outsideMesh.vertices.push_back(segment.controlPoints[1].x);
						outsideMesh.vertices.push_back(segment.controlPoints[1].y);
						outsideMesh.vertices.push_back(0.5f);
						outsideMesh.vertices.push_back(0.0f);
						outsideMesh.vertices.push_back(direction);
						
						outsideMesh.vertices.push_back(segment.controlPoints[2].x);
						outsideMesh.vertices.push_back(segment.controlPoints[2].y);
						outsideMesh.vertices.push_back(1.0f);
						outsideMesh.vertices.push_back(1.0f);
						outsideMesh.vertices.push_back(direction);
						
						outsideMesh.indices.push_back(outsideMesh.indices.size());
						outsideMesh.indices.push_back(outsideMesh.indices.size());
						outsideMesh.indices.push_back(outsideMesh.indices.size());
					}
				}
				else if(segment.type == PathSegment::TypeBezierCubic)
				{
					//Cubics are currently not supported for the outline and should have been converted in a previous step already.
				}
			}
			
			polygon.outlines.push_back(outline);
		}
		
		TriangleMesh insideMesh = TriangulatorBruteForce::Triangulate(polygon);
		uint32_t vertexIndexOffset = insideMesh.vertices.size() / 5;
		insideMesh.vertices.insert(insideMesh.vertices.end(), outsideMesh.vertices.begin(), outsideMesh.vertices.end());
		for(uint32_t index : outsideMesh.indices)
		{
			insideMesh.indices.push_back(index + vertexIndexOffset);
		}
		
		return insideMesh;
	}
}

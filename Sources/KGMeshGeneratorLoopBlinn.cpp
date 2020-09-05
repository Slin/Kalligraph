//
//  KGMeshGeneratorLoopBlinn.cpp
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#include "KGMeshGeneratorLoopBlinn.h"

#include <cmath>
#include <limits>
//#include <iostream>

namespace KG
{
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

	const PathCollection MeshGeneratorLoopBlinn::FilterDegenerateSegments(const PathCollection &paths)
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
						int8_t onLineResult = Math::IsOnLine(newSegment.controlPoints[0], newSegment.controlPoints[2], newSegment.controlPoints[1]);
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
					if(BA.GetDotProduct(BA) <= std::numeric_limits<double>::epsilon())
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

	void CheckForOverlap(std::vector<PathSegment> &oldPathSegments, std::vector<PathSegment> &newPathSegments)
	{
		const PathSegment newSegment = newPathSegments.back();
		for(int i = 0; i < oldPathSegments.size(); i++)
		{
			const PathSegment oldSegment = oldPathSegments[i];
			
			if(Math::AreTrianglesIntersecting(newSegment.controlPoints[0], newSegment.controlPoints[1], newSegment.controlPoints[2], oldSegment.controlPoints[0], oldSegment.controlPoints[1], oldSegment.controlPoints[2]))
			{
				if(Math::GetSquaredTriangleArea(newSegment.controlPoints[0], newSegment.controlPoints[1], newSegment.controlPoints[2]) > Math::GetSquaredTriangleArea(oldSegment.controlPoints[0], oldSegment.controlPoints[1], oldSegment.controlPoints[2]))
				{
					//Subdivide new segment if it's bigger
					
					newPathSegments.pop_back(); //Remove the triangle that gets subdivided
					PathSegment subdividedSegment[2];
					
					subdividedSegment[0].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[0].controlPoints.push_back(newSegment.controlPoints[0]);
					subdividedSegment[0].controlPoints.push_back({0.5f * (newSegment.controlPoints[0].x + newSegment.controlPoints[1].x), 0.5f * (newSegment.controlPoints[0].y + newSegment.controlPoints[1].y)});
					subdividedSegment[0].controlPoints.push_back({0.25f * (newSegment.controlPoints[0].x + newSegment.controlPoints[2].x) + 0.5 * newSegment.controlPoints[1].x, 0.25f * (newSegment.controlPoints[0].y + newSegment.controlPoints[2].y) + 0.5 * newSegment.controlPoints[1].y});
					
					newPathSegments.push_back(subdividedSegment[0]);
					CheckForOverlap(oldPathSegments, newPathSegments);
					
					subdividedSegment[1].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[1].controlPoints.push_back(subdividedSegment[0].controlPoints[2]);
					subdividedSegment[1].controlPoints.push_back({0.5f * (newSegment.controlPoints[1].x + newSegment.controlPoints[2].x), 0.5f * (newSegment.controlPoints[1].y + newSegment.controlPoints[2].y)});
					subdividedSegment[1].controlPoints.push_back(newSegment.controlPoints[2]);
					
					newPathSegments.push_back(subdividedSegment[1]);
					CheckForOverlap(oldPathSegments, newPathSegments);
				}
				else
				{
					//Subdivide old segment if it's bigger
					
					oldPathSegments.pop_back(); //Remove the triangle that gets subdivided
					PathSegment subdividedSegment[2];
					
					subdividedSegment[0].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[0].controlPoints.push_back(oldSegment.controlPoints[0]);
					subdividedSegment[0].controlPoints.push_back({0.5f * (oldSegment.controlPoints[0].x + oldSegment.controlPoints[1].x), 0.5f * (oldSegment.controlPoints[0].y + oldSegment.controlPoints[1].y)});
					subdividedSegment[0].controlPoints.push_back({0.25f * (oldSegment.controlPoints[0].x + oldSegment.controlPoints[2].x) + 0.5 * oldSegment.controlPoints[1].x, 0.25f * (oldSegment.controlPoints[0].y + oldSegment.controlPoints[2].y) + 0.5 * oldSegment.controlPoints[1].y});
					
					oldPathSegments.push_back(subdividedSegment[0]);
					CheckForOverlap(newPathSegments, oldPathSegments);
					
					subdividedSegment[1].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[1].controlPoints.push_back(subdividedSegment[0].controlPoints[2]);
					subdividedSegment[1].controlPoints.push_back({0.5f * (oldSegment.controlPoints[1].x + oldSegment.controlPoints[2].x), 0.5f * (oldSegment.controlPoints[1].y + oldSegment.controlPoints[2].y)});
					subdividedSegment[1].controlPoints.push_back(oldSegment.controlPoints[2]);
					
					oldPathSegments.push_back(subdividedSegment[1]);
					CheckForOverlap(newPathSegments, oldPathSegments);
					
					i += oldPathSegments.size()-1;
				}
			}
		}
	}

	void CheckForOverlap(PathCollection &paths, std::vector<PathSegment> &newSegments)
	{
		for(Path &otherPath : paths.paths)
		{
			for(int i = 0; i < otherPath.segments.size(); i++)
			{
				//Only subdivide for quadratic with quadratic segment triangle intersection for now
				//Cubic segments are not currently supported, but would need special handling here
				if(newSegments.back().type == PathSegment::TypeBezierQuadratic && otherPath.segments[i].type == PathSegment::TypeBezierQuadratic)
				{
					std::vector<PathSegment> oldSegments;
					oldSegments.push_back(otherPath.segments[i]);
					CheckForOverlap(oldSegments, newSegments);
					
					//Update the old segment with it's subdivision if there are any
					if(oldSegments.size() > 1)
					{
						otherPath.segments.erase(otherPath.segments.begin() + i);
						otherPath.segments.insert(otherPath.segments.begin() + i, oldSegments.begin(), oldSegments.end());
					}
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
			//int counter = 0;
			for(const PathSegment &segment : path.segments)
			{
				//std::cout << "handle segment: " << counter << std::endl;
				if(segment.type == PathSegment::TypeBezierCubic)
				{
					//Cubics are currently not supported here and should have been converted in a previous step already.
				}
				else if(segment.type == PathSegment::TypeBezierQuadratic)
				{
					std::vector<PathSegment> newSegments;
					newSegments.push_back(segment);
					
					CheckForOverlap(result, newSegments);
					result.paths.back().segments.insert(result.paths.back().segments.end(), newSegments.begin(), newSegments.end());
				}
				else
				{
					result.paths.back().segments.push_back(segment);
				}
				counter += 1;
			}
			
			//If new path is empty, remove it
			if(result.paths.back().segments.size() == 0)
			{
				result.paths.pop_back();
			}
		}
		
		return result;
	}

	const TriangleMesh MeshGeneratorLoopBlinn::GetMeshForPathCollection(const PathCollection &paths, bool isCCW)
	{
		TriangleMesh outsideMesh;
		outsideMesh.features.push_back(TriangleMesh::VertexFeaturePosition);
		outsideMesh.features.push_back(TriangleMesh::VertexFeatureUV);
		
		PathCollection filteredPaths = DowngradeCubicSegments(paths);
		filteredPaths = FilterDegenerateSegments(filteredPaths);
		filteredPaths = ResolveOverlaps(filteredPaths);
		
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

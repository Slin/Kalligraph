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
	int8_t onLineOther(const Vector2 &A, const Vector2 &B, const Vector2 &C)
	{
		double result = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
		if(std::abs(result) < std::numeric_limits<double>::epsilon()) return 0;
		if(result < 0) return -1;
		return 1;
	}

	void MeshGeneratorLoopBlinn::AddLineSegmentToOutline(BruteForceTriangulator::Outline &outline, const PathSegment &segment)
	{
		Vector2 BA;
		BA.x = segment.controlPoints[1].x - segment.controlPoints[0].x;
		BA.y = segment.controlPoints[1].y - segment.controlPoints[0].y;
		
		if(BA.GetDotProduct(BA) > std::numeric_limits<double>::epsilon())
		{
			outline.points.push_back(segment.controlPoints[1]);
		}
	}

	void MeshGeneratorLoopBlinn::AddQuadraticSegmentToOutline(BruteForceTriangulator::Outline &outline, TriangleMesh &outsideMesh, const PathSegment &segment, const bool &isCCW)
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
			else if((onLineResult < 0 && isCCW) || (onLineResult > 0 && !isCCW))
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
				
				outsideMesh.indices.push_back(outsideMesh.indices.size());
				outsideMesh.indices.push_back(outsideMesh.indices.size());
				outsideMesh.indices.push_back(outsideMesh.indices.size());
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
				
				outsideMesh.indices.push_back(outsideMesh.indices.size());
				outsideMesh.indices.push_back(outsideMesh.indices.size());
				outsideMesh.indices.push_back(outsideMesh.indices.size());
			}
		}
		
		if(needsControlPoint)
		{
			outline.points.push_back(segment.controlPoints[1]);
		}
		if(needsEndPoint)
		{
			outline.points.push_back(segment.controlPoints[2]);
		}
	}

	void MeshGeneratorLoopBlinn::AddCubicSegmentToOutline(BruteForceTriangulator::Outline &outline, TriangleMesh &outsideMesh, const PathSegment &segment, const bool &isCCW)
	{
		//Turn cubic into quadratic
		Vector2 control;
		control.x = -0.25f * segment.controlPoints[0].x + 0.75f * segment.controlPoints[1].x + 0.75f * segment.controlPoints[2].x - 0.25f * segment.controlPoints[3].x;
		control.y = -0.25f * segment.controlPoints[0].y + 0.75f * segment.controlPoints[1].y + 0.75f * segment.controlPoints[2].y - 0.25f * segment.controlPoints[3].y;
		
		PathSegment quadraticSegment;
		quadraticSegment.controlPoints.push_back(segment.controlPoints[0]);
		quadraticSegment.controlPoints.push_back(control);
		quadraticSegment.controlPoints.push_back(segment.controlPoints[3]);
		
		AddQuadraticSegmentToOutline(outline, outsideMesh, quadraticSegment, isCCW);
	}

	const TriangleMesh MeshGeneratorLoopBlinn::GetMeshForPathCollection(const PathCollection &paths, bool isCCW)
	{
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
				if(!isNotFirstSegment)
				{
					outline.points.push_back(segment.controlPoints[0]);
				}
				isNotFirstSegment = true;
				
				if(segment.type == PathSegment::TypeLine)
				{
					AddLineSegmentToOutline(outline, segment);
				}
				if(segment.type == PathSegment::TypeBezierCubic)
				{
					AddCubicSegmentToOutline(outline, outsideMesh, segment, isCCW);
				}
				if(segment.type == PathSegment::TypeBezierQuadratic)
				{
					AddQuadraticSegmentToOutline(outline, outsideMesh, segment, isCCW);
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

//
//  KGBruteForceTriangulator.h
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#ifndef __KG_ConstrainedDelaunayTriangulation_H__
#define __KG_ConstrainedDelaunayTriangulation_H__

#include "KGCommon.h"

namespace KG
{
	class TriangulatorBruteForce
	{
	public:
		class SortedPoint;
		
		class Edge
		{
		public:
			SortedPoint *sortedPoints[2];
			uint8_t triangleCount;
		};
		
		class Triangle
		{
		public:
			SortedPoint *sortedPoints[3];
		};
		
		class SortedPoint
		{
		public:
			uint32_t vertexIndex;
			Vector2 point;
			std::vector<Edge*> edges;
		};
		
		class Outline
		{
		public:
			std::vector<Vector2> points;
		};
		
		class Polygon
		{
		public:
			std::vector<Outline> outlines;
		};
		
		class VisibilityResult
		{
		public:
			bool isBlocked;
			
			Edge *existingFirstEdge;
			Edge *existingSecondEdge;
		};
		
		static TriangleMesh Triangulate(const Polygon &polygon);
		static VisibilityResult CanEdgeSeePoint(Edge *edge, SortedPoint *point, const std::vector<Edge*> &edges, uint32_t outsideEdgeCount);
		static void AddEdgeToEdgeListHandlingIntersections(Edge *edge, std::vector<Edge*> &edges, std::vector<SortedPoint *> &sortedPoints);
	};
}

#endif /* defined(__KG_ConstrainedDelaunayTriangulation_H__) */

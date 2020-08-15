//
//  KGBruteForceTriangulator.cpp
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#include "KGBruteForceTriangulator.h"
#include <cmath>
#include <limits>
#include <iostream>

namespace KG
{
	bool ccw(const Vector2 &A, const Vector2 &B, const Vector2 &C)
	{
		return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
	}

	bool doIntersect(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D)
	{
		return ccw(A,C,D) != ccw(B,C,D) && ccw(A,B,C) != ccw(A,B,D);
	}

	int8_t onLine(const Vector2 &p, const Vector2 &q, const Vector2 &r)
	{
		float result = (r.x - q.x) * (p.y - q.y) - (r.y - q.y) * (p.x - q.x);
		if(std::abs(result) < std::numeric_limits<float>::epsilon()*0.1) return 0;
		if(result < 0) return -1;
		return 1;
	}

	Vector2 getIntersectionPoint(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D)
	{
		float line1[3];
		line1[0] = A.y - B.y;
		line1[1] = B.x - A.x;
		line1[2] = -A.x * B.y + B.x * A.y;
		
		float line2[3];
		line2[0] = C.y - D.y;
		line2[1] = D.x - C.x;
		line2[2] = -C.x * D.y + D.x * C.y;
		
		float d = line1[0] * line2[1] - line1[1] * line2[0];
		if(d == 0) return Vector2{0.0f, 0.0f}; //Prevent dividing with 0, this function should only be called if there is a known intersection, so this shouldn't ever happen...
		
		float dx = line1[2] * line2[1] - line1[1] * line2[2];
		float dy = line1[0] * line2[2] - line1[2] * line2[0];
		
		return Vector2{dx/d, dy/d};
	}

	TriangleMesh BruteForceTriangulator::Triangulate(const Polygon &polygon)
	{
		TriangleMesh mesh;
		
		mesh.features.push_back(TriangleMesh::VertexFeaturePosition);
		mesh.features.push_back(TriangleMesh::VertexFeatureUV);
		
		std::vector<Edge*> edges;
		edges.reserve(polygon.outlines[0].points.size() * 3);
		std::vector<SortedPoint*> sortedPoints;
		
		//Generate data structures from input data
		uint32_t previousPointIndex = 0;
		for(const Outline &outline : polygon.outlines)
		//const Outline &outline = polygon.outlines[0];
		{
			SortedPoint *firstVertex = nullptr;
			uint32_t pointIndex = 0;
			Edge *edge = new Edge();
			edge->triangleCount = 1; //Start outlines with a count of 1, to later have a count of 2 and be skipped without having to check if they are part of the outline
			for(const Vector2 &point : outline.points)
			{
				//Skip duplicated points (There could also still be douplicates that aren't directly connected, should be much less likely though)
				if(firstVertex)
				{
					float diffX = point.x - sortedPoints[previousPointIndex]->point.x;
					float diffY = point.y - sortedPoints[previousPointIndex]->point.y;
					if(std::abs(diffX) < std::numeric_limits<float>::epsilon() && std::abs(diffY) < std::numeric_limits<float>::epsilon())
					{
						pointIndex += 1;
						continue;
					}
					
					diffX = point.x - firstVertex->point.x;
					diffY = point.y - firstVertex->point.y;
					if(std::abs(diffX) < std::numeric_limits<float>::epsilon() && std::abs(diffY) < std::numeric_limits<float>::epsilon())
					{
						pointIndex += 1;
						continue;
					}
				}
				
				//Create sorted points and edges
				SortedPoint *sortedPoint = new SortedPoint();
				sortedPoint->vertexIndex = sortedPoints.size();
				sortedPoint->point = point;
				sortedPoints.push_back(sortedPoint);
				
				previousPointIndex = sortedPoints.size()-1; //Needs to be stored here, cause the next method may add new points to end of the list, but those are not possible duplicates
				
				if(firstVertex)
				{
					edge->sortedPoints[1] = sortedPoint;
					AddEdgeToEdgeListHandlingCrossings(edge, edges, sortedPoints);
					
					edge = new Edge();
					edge->triangleCount = 1; //Start outlines with a count of 1, to later have a count of 2 and be skipped without having to check if they are part of the outline
				}
				else
				{
					firstVertex = sortedPoint;
				}
				edge->sortedPoints[0] = sortedPoint;
				
				pointIndex += 1;
			}
			
			//Close outline by connecting last vertex to first
			if(firstVertex)
			{
				edge->sortedPoints[1] = firstVertex;
				AddEdgeToEdgeListHandlingCrossings(edge, edges, sortedPoints);
			}
		}
		
		//Create vertices from point list
		for(SortedPoint *sortedPoint : sortedPoints)
		{
			//Create vertices
			mesh.vertices.push_back(sortedPoint->point.x);
			mesh.vertices.push_back(sortedPoint->point.y);
			
			mesh.vertices.push_back(1.0f);
			mesh.vertices.push_back(0.0f);
			mesh.vertices.push_back(0.0f);
		}
		
		//Brute force the triangles
		std::vector<Triangle*> triangles;
		uint32_t originalEdgeCount = edges.size();
		for(int i = 0; i < edges.size(); i++)
		{
			Edge *edge = edges[i];
			
			//Every inside edge needs to be part of two triangles, otherwise there is a hole. Can't be more than two either though
			if(edge->triangleCount >= 2) continue;
			
			//TODO: Remove
//			if(i >= 81)
			{
				std::cout << "edge " << i << ": " << edge->sortedPoints[0]->vertexIndex << " (" << edge->sortedPoints[0]->point.x << ", " << edge->sortedPoints[0]->point.y << ") - " << edge->sortedPoints[1]->vertexIndex << " (" << edge->sortedPoints[1]->point.x << ", " << edge->sortedPoints[1]->point.y << ")" << std::endl;
			}
			
			int pointCounter = 0;
			for(SortedPoint *sortedPoint : sortedPoints)
			{
				//if(pointCounter >= 68)
				{
					std::cout << "point " << pointCounter << ": (" << sortedPoint->point.x << ", " << sortedPoint->point.y << ")" << std::endl;
				}
				pointCounter += 1;
				
				if(sortedPoint == edge->sortedPoints[0] || sortedPoint == edge->sortedPoints[1]) continue;
				
				VisibilityResult visibilityResult = CanEdgeSeePoint(edge, sortedPoint, edges, originalEdgeCount);
				if(!visibilityResult.isBlocked)
				{
					std::cout << "found point " << pointCounter-1 << std::endl;
					
					if(!visibilityResult.existingFirstEdge)
					{
						Edge *edge1 = new Edge();
						edge1->sortedPoints[0] = edge->sortedPoints[0];
						edge1->sortedPoints[1] = sortedPoint;
						edge1->triangleCount = 1;
						edges.push_back(edge1);
						
						edge1->sortedPoints[0]->edges.push_back(edge1);
						edge1->sortedPoints[1]->edges.push_back(edge1);
					}
					else
					{
						visibilityResult.existingFirstEdge->triangleCount += 1;
					}
					
					if(!visibilityResult.existingSecondEdge)
					{
						Edge *edge2 = new Edge();
						edge2->sortedPoints[0] = edge->sortedPoints[1];
						edge2->sortedPoints[1] = sortedPoint;
						edge2->triangleCount = 1;
						edges.push_back(edge2);
						
						edge2->sortedPoints[0]->edges.push_back(edge2);
						edge2->sortedPoints[1]->edges.push_back(edge2);
					}
					else
					{
						visibilityResult.existingSecondEdge->triangleCount += 1;
					}
					
					Triangle *triangle = new Triangle();
					triangle->sortedPoints[0] = edge->sortedPoints[0];
					triangle->sortedPoints[1] = edge->sortedPoints[1];
					triangle->sortedPoints[2] = sortedPoint;
					triangles.push_back(triangle);
					
					break; //Found a point to connect the edge to, no need to check more points against it
				}
			}
		}

		for(Triangle *triangle : triangles)
		{
			mesh.indices.push_back(triangle->sortedPoints[0]->vertexIndex);
			mesh.indices.push_back(triangle->sortedPoints[1]->vertexIndex);
			mesh.indices.push_back(triangle->sortedPoints[2]->vertexIndex);
		}
		
		return mesh;
	}

	BruteForceTriangulator::VisibilityResult BruteForceTriangulator::CanEdgeSeePoint(Edge *edge, SortedPoint *point, const std::vector<Edge*> &edges, uint32_t outsideEdgeCount)
	{
		VisibilityResult result;
 		result.isBlocked = true;
		result.existingFirstEdge = nullptr;
		result.existingSecondEdge = nullptr;
		
		//Stop if points are all along one line
		if(onLine(point->point, edge->sortedPoints[0]->point, edge->sortedPoints[1]->point) == 0)
		{
			return result;
		}
		
		//All new edges need to either be existing ones OR be inside the polygon (that don't cross any other, but that's handled further down)
		Vector2 midPointEdge[2];
		midPointEdge[0].x = (point->point.x + edge->sortedPoints[0]->point.x) / 2.0f;
		midPointEdge[0].y = (point->point.y + edge->sortedPoints[0]->point.y) / 2.0f;
		midPointEdge[1].x = (point->point.x + edge->sortedPoints[1]->point.x) / 2.0f;
		midPointEdge[1].y = (point->point.y + edge->sortedPoints[1]->point.y) / 2.0f;

		Edge *existingFirstEdge = nullptr;
		Edge *existingSecondEdge = nullptr;
		
		uint32_t insideCounter[2] = {0, 0};
		for(Edge *otherEdge : edges)
		{
			//Check if ray from midPoint along the x axis hits the edge, but only for the original outline edges
			if(outsideEdgeCount > 0)
			{
				outsideEdgeCount -= 1;
				
				int higherPointIndex = otherEdge->sortedPoints[0]->point.y > otherEdge->sortedPoints[1]->point.y? 0 : 1;
				int lowerPointIndex = higherPointIndex == 0? 1 : 0;
				
				int rightPointIndex = otherEdge->sortedPoints[0]->point.x > otherEdge->sortedPoints[1]->point.x? 0 : 1;
				int leftPointIndex = rightPointIndex == 0? 1 : 0;
				
				for(int i = 0; i < 2; i++)
				{
					//Midpoint is not higher or lower than the edge and not to the right of it's right point
					if(midPointEdge[i].y > otherEdge->sortedPoints[lowerPointIndex]->point.y && midPointEdge[i].y <= otherEdge->sortedPoints[higherPointIndex]->point.y && midPointEdge[i].x < otherEdge->sortedPoints[rightPointIndex]->point.x)
					{
						if(midPointEdge[i].x < otherEdge->sortedPoints[leftPointIndex]->point.x)
						{
							//If on the left of the left point of the edge, the check definitely hits it
							insideCounter[i] += 1;
						}
						else if(onLine(midPointEdge[i], otherEdge->sortedPoints[higherPointIndex]->point, otherEdge->sortedPoints[lowerPointIndex]->point) == -1)
						{
							insideCounter[i] += 1;
						}
					}
				}
			}
			
			if(edge == otherEdge) continue; //Skip if it's the same edge
			
			//All good if the edge already exists (first edge)
			if((edge->sortedPoints[0] == otherEdge->sortedPoints[0] && point == otherEdge->sortedPoints[1]) || (edge->sortedPoints[0] == otherEdge->sortedPoints[1] && point == otherEdge->sortedPoints[0]))
			{
				//But only if it's not already part of two triangles!
				if(otherEdge->triangleCount >= 2)
				{
					return result;
				}
				
				existingFirstEdge = otherEdge;
				continue;
			}
			
			//All good if the edge already exists (second edge)
			if((edge->sortedPoints[1] == otherEdge->sortedPoints[0] && point == otherEdge->sortedPoints[1]) || (edge->sortedPoints[1] == otherEdge->sortedPoints[1] && point == otherEdge->sortedPoints[0]))
			{
				//But only if it's not already part of two triangles!
				if(otherEdge->triangleCount >= 2)
				{
					return result;
				}
				
				existingSecondEdge = otherEdge;
				continue;
			}
			
			//If the point is one of the other edges vertices it's good to go even though it is an intersection
			if(point == otherEdge->sortedPoints[0] || point == otherEdge->sortedPoints[1])
			{
				//Wait, can't skip if there is a secondary connection to edge
				SortedPoint *otherPoint = otherEdge->sortedPoints[0];
				if(otherPoint == point) otherPoint = otherEdge->sortedPoints[1];
				for(Edge *secondaryEdge : otherPoint->edges)
				{
					if(secondaryEdge == otherEdge) continue;
					
					if(secondaryEdge->sortedPoints[0] == edge->sortedPoints[0] || secondaryEdge->sortedPoints[0] == edge->sortedPoints[1] || secondaryEdge->sortedPoints[1] == edge->sortedPoints[0] || secondaryEdge->sortedPoints[1] == edge->sortedPoints[1])
					{
						SortedPoint *secondaryEdgePoint = secondaryEdge->sortedPoints[0];
						if(secondaryEdgePoint == otherPoint) secondaryEdgePoint = secondaryEdge->sortedPoints[1];
						
						//Return as blocking if the edges are colinear
						if(onLine(point->point, otherPoint->point, secondaryEdgePoint->point) == 0)
						{
							return result;
						}
					}
				}
				
				continue;
			}
			
			//Check if the new edges would intersect this edge
			if(edge->sortedPoints[0] != otherEdge->sortedPoints[1] && edge->sortedPoints[0] != otherEdge->sortedPoints[0] && doIntersect(edge->sortedPoints[0]->point, point->point, otherEdge->sortedPoints[0]->point, otherEdge->sortedPoints[1]->point))
			{
				return result;
			}
			
			if(edge->sortedPoints[1] != otherEdge->sortedPoints[1] && edge->sortedPoints[1] != otherEdge->sortedPoints[0] && doIntersect(edge->sortedPoints[1]->point, point->point, otherEdge->sortedPoints[0]->point, otherEdge->sortedPoints[1]->point))
			{
				return result;
			}
		}
		
		//Return true if triangle is inside the polygon
		if((existingFirstEdge || insideCounter[0] % 2 == 1) && (existingSecondEdge || insideCounter[1] % 2 == 1))
		{
			result.isBlocked = false;
			result.existingFirstEdge = existingFirstEdge;
			result.existingSecondEdge = existingSecondEdge;
			return result;
		}
		
		return result;
	}

	void BruteForceTriangulator::AddEdgeToEdgeListHandlingCrossings(Edge *edge, std::vector<Edge*> &edges, std::vector<SortedPoint *> &sortedPoints)
	{
		std::vector<SortedPoint *> newPoints;
		for(int i = 0; i < edges.size(); i++)
		{
			Edge *otherEdge = edges[i];
			
			//No intersection if edges share a point
			if(edge->sortedPoints[0] == otherEdge->sortedPoints[0] || edge->sortedPoints[1] == otherEdge->sortedPoints[1] || edge->sortedPoints[0] == otherEdge->sortedPoints[1] || edge->sortedPoints[1] == otherEdge->sortedPoints[0]) continue;
			
			if(doIntersect(edge->sortedPoints[0]->point, edge->sortedPoints[1]->point, otherEdge->sortedPoints[0]->point, otherEdge->sortedPoints[1]->point))
			{
				//Create a new vertex at the intersection point
				SortedPoint *newPoint = new SortedPoint();
				newPoint->vertexIndex = sortedPoints.size();
				newPoint->point = getIntersectionPoint(edge->sortedPoints[0]->point, edge->sortedPoints[1]->point, otherEdge->sortedPoints[0]->point, otherEdge->sortedPoints[1]->point);
				sortedPoints.push_back(newPoint);
				
				std::cout << "split point " << newPoint->vertexIndex << ": (" << newPoint->point.x << ", " << newPoint->point.y << "), inserted into edge " << i << std::endl;
				
				//Add to new point list, which will later be used to create the new edges
				newPoints.push_back(newPoint);
				
				//Split otherEdge here as it will always be split into two
				otherEdge->sortedPoints[0]->edges.clear();
				otherEdge->sortedPoints[1]->edges.clear();
				
				Edge *newEdge = new Edge();
				newEdge->triangleCount = otherEdge->triangleCount;
				newEdge->sortedPoints[0] = newPoint;
				newEdge->sortedPoints[1] = otherEdge->sortedPoints[1];
				otherEdge->sortedPoints[1] = newPoint;
				
				otherEdge->sortedPoints[0]->edges.push_back(otherEdge);
				otherEdge->sortedPoints[1]->edges.push_back(otherEdge);
				newEdge->sortedPoints[0]->edges.push_back(newEdge);
				newEdge->sortedPoints[1]->edges.push_back(newEdge);
				
				//TODO: push_back() instead, if order really doesn't matter
				i++;
				edges.insert(edges.begin()+i, newEdge);
			}
		}
		
		if(newPoints.size() > 0)
		{
			//The edge may have been cut into many new edges. Need to sort points along the edge (distance from first point, add last point to the list to not need extra handling)
			newPoints.push_back(edge->sortedPoints[1]);
			sort(newPoints.begin(), newPoints.end(), [edge](const SortedPoint *a, const SortedPoint *b) {
				Vector2 distanceA;
				distanceA.x = a->point.x - edge->sortedPoints[0]->point.x;
				distanceA.y = a->point.y - edge->sortedPoints[0]->point.y;
				
				Vector2 distanceB;
				distanceB.x = b->point.x - edge->sortedPoints[0]->point.x;
				distanceB.y = b->point.y - edge->sortedPoints[0]->point.y;
				
				return distanceA.GetDotProduct(distanceA) < distanceB.GetDotProduct(distanceB);
			});
			
			//Connect the points to form new edges
			for(int i = 0; i < newPoints.size(); i++)
			{
				SortedPoint *sortedPoint = newPoints[i];
				
				uint8_t triangleCount = edge->triangleCount;
				edge->sortedPoints[1] = sortedPoint;
				
				edges.push_back(edge);
				edge->sortedPoints[0]->edges.push_back(edge);
				edge->sortedPoints[1]->edges.push_back(edge);
				
				if(i < sortedPoints.size() - 1)
				{
					edge = new Edge();
					edge->triangleCount = triangleCount;
					edge->sortedPoints[0] = sortedPoint;
				}
			}
		}
		else
		{
			edges.push_back(edge);
			edge->sortedPoints[0]->edges.push_back(edge);
			edge->sortedPoints[1]->edges.push_back(edge);
		}
	}
}

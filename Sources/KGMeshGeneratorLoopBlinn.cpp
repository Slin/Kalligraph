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

	//Find winding order
	//This assumes no intersections in any of the paths and will find the polygon winding order for each
	const PathCollection MeshGeneratorLoopBlinn::FindWindingOrder(const PathCollection &paths)
	{
		PathCollection result;
		
		uint32_t segmentIndex = 0;
		for(const Path &path : paths.paths)
		{
			result.paths.push_back(Path());
			
			for(const PathSegment &segment : path.segments)
			{
				Vector2 segmentCenter;
				segmentCenter.x = 0;
				segmentCenter.y = 0;
				for(const Vector2 &point : segment.controlPoints)
				{
					segmentCenter.x += point.x;
					segmentCenter.y += point.y;
				}
				segmentCenter.x /= segment.controlPoints.size();
				segmentCenter.y /= segment.controlPoints.size();
				
				uint32_t otherSegmentIndex = 0;
				uint32_t insideCounter = 0;
				for(const Path &otherPath : paths.paths)
				{
					for(const PathSegment &otherSegment : otherPath.segments)
					{
						if(segmentIndex == otherSegmentIndex)
						{
							otherSegmentIndex += 1;
							continue;
						}
						
						int higherPointIndex = otherSegment.controlPoints[0].y > otherSegment.controlPoints.back().y? 0 : (otherSegment.controlPoints.size() - 1);
						int lowerPointIndex = higherPointIndex == 0? (otherSegment.controlPoints.size() - 1) : 0;
						
						int rightPointIndex = otherSegment.controlPoints[0].x > otherSegment.controlPoints.back().x? 0 : (otherSegment.controlPoints.size() - 1);
						int leftPointIndex = rightPointIndex == 0? (otherSegment.controlPoints.size() - 1) : 0;
						
						//Midpoint is not higher or lower than the edge and not to the right of it's right point
						if(segmentCenter.y > otherSegment.controlPoints[lowerPointIndex].y && segmentCenter.y <= otherSegment.controlPoints[higherPointIndex].y && segmentCenter.x > otherSegment.controlPoints[leftPointIndex].x)
						{
/*							if(segmentCenter.x > otherSegment.controlPoints[rightPointIndex].x)
							{
								//If on the left of the left point of the edge, the check definitely hits it
								insideCounter += 1;
							}
							else */if(Math::IsOnLine(otherSegment.controlPoints[higherPointIndex], otherSegment.controlPoints[lowerPointIndex], segmentCenter) == 1)
							{
								insideCounter += 1;
							}
						}
						
						otherSegmentIndex += 1;
					}
				}
				
				PathSegment newSegment = segment;
				newSegment.isFilledOutside = (insideCounter % 2) == 0;
				
				if(segment.type == PathSegment::TypeBezierQuadratic)
				{
					int higherPointIndex = segment.controlPoints[0].y > segment.controlPoints.back().y? 0 : (segment.controlPoints.size() - 1);
					int lowerPointIndex = higherPointIndex == 0? (segment.controlPoints.size() - 1) : 0;
					if(Math::IsOnLine(segment.controlPoints[higherPointIndex], segment.controlPoints[lowerPointIndex], segment.controlPoints[1]) == 1)
					{
						newSegment.isFilledOutside = !newSegment.isFilledOutside;
					}
				}
				
				
				result.paths.back().segments.push_back(newSegment);
				
				segmentIndex += 1;
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

	//Checks if quadratic curve and line segment intersect and splits them if they do.
	void MeshGeneratorLoopBlinn::ResolveQuadraticLineIntersection(std::vector<PathSegment> &iteratedPathSegments, std::vector<PathSegment> &otherPathSegments)
	{
		const PathSegment otherSegment = otherPathSegments.back();
		for(int i = 0; i < iteratedPathSegments.size(); i++)
		{
			const PathSegment iteratedSegment = iteratedPathSegments[i];
			
			const PathSegment &quadraticSegment = iteratedSegment.type == PathSegment::TypeBezierQuadratic?iteratedSegment : otherSegment;
			const PathSegment &lineSegment = iteratedSegment.type == PathSegment::TypeBezierQuadratic?otherSegment : iteratedSegment;
			
			//Other than with line line intersection, there can be another intersection even if one of the control points is shared
			std::vector<double> intersectionCoefficients = Math::GetQuadraticCurveAndLineSegmentIntersectionCoefficients(quadraticSegment.controlPoints[0], quadraticSegment.controlPoints[1], quadraticSegment.controlPoints[2], lineSegment.controlPoints[0], lineSegment.controlPoints[1]);
			if(intersectionCoefficients.size() > 0)
			{
				bool hasTwoIntersections = intersectionCoefficients.size() > 1;
				
				otherPathSegments.pop_back(); //Remove the segment that gets split
				iteratedPathSegments.erase(iteratedPathSegments.begin() + i); //Remove the segment that gets split
				
				//Calculate points by inserting t into squared curve equation
				Vector2 intersectionPoints[2];
				Vector2 controlPoints[3];
				
				double t = intersectionCoefficients[0];
				double s = 1.0 - t;
				
				intersectionPoints[0].x = s * s * quadraticSegment.controlPoints[0].x + 2.0 * t * s * quadraticSegment.controlPoints[1].x + t * t * quadraticSegment.controlPoints[2].x;
				intersectionPoints[0].y = s * s * quadraticSegment.controlPoints[0].y + 2.0 * t * s * quadraticSegment.controlPoints[1].y + t * t * quadraticSegment.controlPoints[2].y;
				
				controlPoints[0].x = s * quadraticSegment.controlPoints[0].x + t * quadraticSegment.controlPoints[1].x;
				controlPoints[0].y = s * quadraticSegment.controlPoints[0].y + t * quadraticSegment.controlPoints[1].y;
				
				controlPoints[1].x = s * quadraticSegment.controlPoints[1].x + t * quadraticSegment.controlPoints[2].x;
				controlPoints[1].y = s * quadraticSegment.controlPoints[1].y + t * quadraticSegment.controlPoints[2].y;
				
				if(hasTwoIntersections)
				{
					t = (intersectionCoefficients[1] - t) / (1.0 - t);
					s = 1.0 - t;
					
					intersectionPoints[1].x = s * s * intersectionPoints[0].x + 2.0 * t * s * controlPoints[1].x + t * t * quadraticSegment.controlPoints[2].x;
					intersectionPoints[1].y = s * s * intersectionPoints[0].y + 2.0 * t * s * controlPoints[1].y + t * t * quadraticSegment.controlPoints[2].y;
					
					controlPoints[2].x = s * controlPoints[1].x + t * quadraticSegment.controlPoints[2].x;
					controlPoints[2].y = s * controlPoints[1].y + t * quadraticSegment.controlPoints[2].y;
					
					controlPoints[1].x = s * intersectionPoints[0].x + t * controlPoints[1].x;
					controlPoints[1].y = s * intersectionPoints[0].y + t * controlPoints[1].y;
				}
				
				//Order intersection points along the line segment
				Vector2 lineIntersectionPoints[2];
				if(!hasTwoIntersections)
				{
					lineIntersectionPoints[0] = intersectionPoints[0];
				}
				else
				{
					Vector2 BA;
					BA.x = intersectionPoints[0].x - lineSegment.controlPoints[0].x;
					BA.y = intersectionPoints[0].y - lineSegment.controlPoints[0].y;
					
					Vector2 CA;
					CA.x = intersectionPoints[1].x - lineSegment.controlPoints[0].x;
					CA.y = intersectionPoints[1].y - lineSegment.controlPoints[0].y;
					
					if(BA.GetDotProduct(BA) < CA.GetDotProduct(CA))
					{
						lineIntersectionPoints[0] = intersectionPoints[0];
						lineIntersectionPoints[1] = intersectionPoints[1];
					}
					else
					{
						lineIntersectionPoints[0] = intersectionPoints[1];
						lineIntersectionPoints[1] = intersectionPoints[0];
					}
				}
				
				//Put together the new segments
				PathSegment subdividedSegment[6];
				
				//Quadratic segments
				subdividedSegment[0].type = PathSegment::TypeBezierQuadratic;
				subdividedSegment[0].controlPoints.push_back(quadraticSegment.controlPoints[0]);
				subdividedSegment[0].controlPoints.push_back(controlPoints[0]);
				subdividedSegment[0].controlPoints.push_back(intersectionPoints[0]);
				
				subdividedSegment[1].type = PathSegment::TypeBezierQuadratic;
				subdividedSegment[1].controlPoints.push_back(intersectionPoints[0]);
				subdividedSegment[1].controlPoints.push_back(controlPoints[1]);
				subdividedSegment[1].controlPoints.push_back(hasTwoIntersections? intersectionPoints[1] : quadraticSegment.controlPoints[2]);
				
				if(hasTwoIntersections)
				{
					subdividedSegment[2].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[2].controlPoints.push_back(intersectionPoints[1]);
					subdividedSegment[2].controlPoints.push_back(controlPoints[2]);
					subdividedSegment[2].controlPoints.push_back(quadraticSegment.controlPoints[2]);
				}
				
				//Line segments
				subdividedSegment[3].type = PathSegment::TypeLine;
				subdividedSegment[3].controlPoints.push_back(lineSegment.controlPoints[0]);
				subdividedSegment[3].controlPoints.push_back(lineIntersectionPoints[0]);
				
				subdividedSegment[4].type = PathSegment::TypeLine;
				subdividedSegment[4].controlPoints.push_back(intersectionPoints[0]);
				subdividedSegment[4].controlPoints.push_back(hasTwoIntersections? lineIntersectionPoints[1] : lineSegment.controlPoints[1]);
				
				if(hasTwoIntersections)
				{
					subdividedSegment[5].type = PathSegment::TypeLine;
					subdividedSegment[5].controlPoints.push_back(lineIntersectionPoints[1]);
					subdividedSegment[5].controlPoints.push_back(lineSegment.controlPoints[1]);
				}
				
				//Insert new segments into the lists
				if(otherSegment.type == PathSegment::TypeBezierQuadratic)
				{
					otherPathSegments.push_back(subdividedSegment[0]);
					otherPathSegments.push_back(subdividedSegment[1]);
					
					iteratedPathSegments.insert(iteratedPathSegments.begin() + i++, subdividedSegment[3]);
					iteratedPathSegments.insert(iteratedPathSegments.begin() + i, subdividedSegment[4]);
					
					if(hasTwoIntersections)
					{
						otherPathSegments.push_back(subdividedSegment[2]);
						iteratedPathSegments.insert(iteratedPathSegments.begin() + ++i, subdividedSegment[5]);
					}
				}
				else
				{
					otherPathSegments.push_back(subdividedSegment[3]);
					otherPathSegments.push_back(subdividedSegment[4]);
					
					iteratedPathSegments.insert(iteratedPathSegments.begin() + i++, subdividedSegment[0]);
					iteratedPathSegments.insert(iteratedPathSegments.begin() + i, subdividedSegment[1]);
					
					if(hasTwoIntersections)
					{
						otherPathSegments.push_back(subdividedSegment[5]);
						iteratedPathSegments.insert(iteratedPathSegments.begin() + ++i, subdividedSegment[2]);
					}
				}
			}
		}
	}


	//Checks if two quadratic curves intersect and splits them if they do.
	void MeshGeneratorLoopBlinn::ResolveQuadraticQuadraticIntersection(std::vector<PathSegment> &iteratedPathSegments, std::vector<PathSegment> &otherPathSegments)
	{
		const PathSegment otherSegment = otherPathSegments.back();
		for(int r = 0; r < iteratedPathSegments.size(); r++)
		{
			const PathSegment iteratedSegment = iteratedPathSegments[r];
			
			//Other than with line line intersection, there can be another intersection even if one of the control points is shared
			std::vector<double> intersectionCoefficients = Math::GetQuadraticCurveAndQuadraticCurveIntersectionCoefficients(iteratedSegment.controlPoints[0], iteratedSegment.controlPoints[1], iteratedSegment.controlPoints[2], otherSegment.controlPoints[0], otherSegment.controlPoints[1], otherSegment.controlPoints[2]);
			if(intersectionCoefficients.size() > 0)
			{
				otherPathSegments.pop_back(); //Remove the segment that gets split
				iteratedPathSegments.erase(iteratedPathSegments.begin() + r); //Remove the segment that gets split
				
				//Order intersection points along the second curve
				int otherSegmentIntersectionIndices[5] = {-1, -1, -1, -1, -1};
				int n = 0;
				for(int i = 1; i < intersectionCoefficients.size(); i += 2)
				{
					for(int l = 0; l < 4; l++)
					{
						if(otherSegmentIntersectionIndices[l] < 0)
						{
							otherSegmentIntersectionIndices[l] = n;
							break;
						}
						else
						{
							if(intersectionCoefficients[i] < intersectionCoefficients[otherSegmentIntersectionIndices[l]])
							{
								for(int m = 3; m > l; m--)
								{
									otherSegmentIntersectionIndices[m] = otherSegmentIntersectionIndices[m-1];
								}
								otherSegmentIntersectionIndices[l] = n;
								break;
							}
						}
					}
					
					n += 1;
				}
				
				//Calculate points by inserting t into squared curve equation
				Vector2 intersectionPoints[4];
				Vector2 controlPoints[10];
				
				double t = intersectionCoefficients[0];
				double s = 1.0 - t;
				
				intersectionPoints[0].x = s * s * iteratedSegment.controlPoints[0].x + 2.0 * t * s * iteratedSegment.controlPoints[1].x + t * t * iteratedSegment.controlPoints[2].x;
				intersectionPoints[0].y = s * s * iteratedSegment.controlPoints[0].y + 2.0 * t * s * iteratedSegment.controlPoints[1].y + t * t * iteratedSegment.controlPoints[2].y;
				
				controlPoints[0].x = s * iteratedSegment.controlPoints[0].x + t * iteratedSegment.controlPoints[1].x;
				controlPoints[0].y = s * iteratedSegment.controlPoints[0].y + t * iteratedSegment.controlPoints[1].y;
				
				controlPoints[1].x = s * iteratedSegment.controlPoints[1].x + t * iteratedSegment.controlPoints[2].x;
				controlPoints[1].y = s * iteratedSegment.controlPoints[1].y + t * iteratedSegment.controlPoints[2].y;
				
				for(int i = 2; i < intersectionCoefficients.size(); i += 2)
				{
					t = (intersectionCoefficients[i] - t) / (1.0 - t);
					s = 1.0 - t;
					
					intersectionPoints[i/2].x = s * s * intersectionPoints[i/2-1].x + 2.0 * t * s * controlPoints[i/2].x + t * t * iteratedSegment.controlPoints[2].x;
					intersectionPoints[i/2].y = s * s * intersectionPoints[i/2-1].y + 2.0 * t * s * controlPoints[i/2].y + t * t * iteratedSegment.controlPoints[2].y;
					
					controlPoints[i].x = s * controlPoints[i-1].x + t * iteratedSegment.controlPoints[2].x;
					controlPoints[i].y = s * controlPoints[i-1].y + t * iteratedSegment.controlPoints[2].y;
					
					controlPoints[i-1].x = s * intersectionPoints[i/2-1].x + t * controlPoints[i-1].x;
					controlPoints[i-1].y = s * intersectionPoints[i/2-1].y + t * controlPoints[i-1].y;
				}
				
				double o = intersectionCoefficients[otherSegmentIntersectionIndices[0] * 2 + 1];
				double v = 1.0 - o;
				
				controlPoints[4].x = v * otherSegment.controlPoints[0].x + o * otherSegment.controlPoints[1].x;
				controlPoints[4].y = v * otherSegment.controlPoints[0].y + o * otherSegment.controlPoints[1].y;
				
				controlPoints[5].x = v * otherSegment.controlPoints[1].x + o * otherSegment.controlPoints[2].x;
				controlPoints[5].y = v * otherSegment.controlPoints[1].y + o * otherSegment.controlPoints[2].y;
				
				for(int i = 2; i < intersectionCoefficients.size(); i += 2)
				{
					o = (intersectionCoefficients[otherSegmentIntersectionIndices[i/2] * 2 + 1] - o) / (1.0 - o);
					v = 1.0 - o;
					
					controlPoints[4+i].x = v * controlPoints[4+i-1].x + o * otherSegment.controlPoints[2].x;
					controlPoints[4+i].y = v * controlPoints[4+i-1].y + o * otherSegment.controlPoints[2].y;
					
					controlPoints[4+i-1].x = v * intersectionPoints[otherSegmentIntersectionIndices[i/2 - 1]].x + o * controlPoints[4+i-1].x;
					controlPoints[4+i-1].y = v * intersectionPoints[otherSegmentIntersectionIndices[i/2 - 1]].y + o * controlPoints[4+i-1].y;
				}
				
				
				//Put together the new segments
				PathSegment subdividedSegment;
				subdividedSegment.type = PathSegment::TypeBezierQuadratic;
				
				int numberOfIntersections = intersectionCoefficients.size() / 2;
				for(int i = 0; i <= numberOfIntersections; i++)
				{
					subdividedSegment.controlPoints.clear();
					subdividedSegment.controlPoints.push_back(i==0?iteratedSegment.controlPoints[0]:intersectionPoints[i-1]);
					subdividedSegment.controlPoints.push_back(controlPoints[i]);
					subdividedSegment.controlPoints.push_back(i==numberOfIntersections?iteratedSegment.controlPoints[2]:intersectionPoints[i]);
					
					//Insert new segment into the list
					iteratedPathSegments.insert(iteratedPathSegments.begin() + r++, subdividedSegment);
					
					
					int intersectionIndex = otherSegmentIntersectionIndices[i];
					subdividedSegment.controlPoints.clear();
					subdividedSegment.controlPoints.push_back(i==0?otherSegment.controlPoints[0]:intersectionPoints[otherSegmentIntersectionIndices[i-1]]);
					subdividedSegment.controlPoints.push_back(controlPoints[4 + i]);
					subdividedSegment.controlPoints.push_back(i==numberOfIntersections?otherSegment.controlPoints[2]:intersectionPoints[intersectionIndex]);
					
					//Insert new segment into the list
					otherPathSegments.push_back(subdividedSegment);
				}
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
						std::vector<PathSegment> oldSegments;
						oldSegments.push_back(otherPath.segments[i]);
						
						//Handle Line - Line segment intersection
						if(segment.type == PathSegment::TypeLine && otherPath.segments[i].type == PathSegment::TypeLine)
						{
							ResolveLineLineIntersection(newSegments, oldSegments);
						}
						else if((segment.type == PathSegment::TypeBezierQuadratic && otherPath.segments[i].type == PathSegment::TypeLine) || (segment.type == PathSegment::TypeLine && otherPath.segments[i].type == PathSegment::TypeBezierQuadratic))
						{
							ResolveQuadraticLineIntersection(newSegments, oldSegments);
						}
						else if(segment.type == PathSegment::TypeBezierQuadratic && otherPath.segments[i].type == PathSegment::TypeBezierQuadratic)
						{
							ResolveQuadraticQuadraticIntersection(newSegments, oldSegments);
						}
						
						//Update the old segment if it was split
						if(oldSegments.size() > 1)
						{
							otherPath.segments.erase(otherPath.segments.begin() + i);
							otherPath.segments.insert(otherPath.segments.begin() + i, oldSegments.begin(), oldSegments.end());
							i += oldSegments.size() - 1;
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
	void MeshGeneratorLoopBlinn::ResolveQuadraticQuadraticOverlap(std::vector<PathSegment> &iteratedPathSegments, std::vector<PathSegment> &otherPathSegments, double minTriangleArea)
	{
		PathSegment otherSegment = otherPathSegments.back();
		for(int i = 0; i < iteratedPathSegments.size(); i++)
		{
			const PathSegment iteratedSegment = iteratedPathSegments[i];
			
			if(Math::AreTrianglesIntersecting(otherSegment.controlPoints[0], otherSegment.controlPoints[1], otherSegment.controlPoints[2], iteratedSegment.controlPoints[0], iteratedSegment.controlPoints[1], iteratedSegment.controlPoints[2]))
			{
				double triangleSize[2];
				triangleSize[0] = Math::GetSquaredTriangleArea(otherSegment.controlPoints[0], otherSegment.controlPoints[1], otherSegment.controlPoints[2]);
				triangleSize[1] = Math::GetSquaredTriangleArea(iteratedSegment.controlPoints[0], iteratedSegment.controlPoints[1], iteratedSegment.controlPoints[2]);
				
				if(triangleSize[0] < minTriangleArea || triangleSize[1] < minTriangleArea)
				{
					return;
				}
				
				if(triangleSize[0] > triangleSize[1])
				{
					//Subdivide other segment if it's bigger
					
					otherPathSegments.pop_back(); //Remove the triangle that gets subdivided
					PathSegment subdividedSegment[2];
					
					subdividedSegment[0].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[0].controlPoints.push_back(otherSegment.controlPoints[0]);
					subdividedSegment[0].controlPoints.push_back({0.5f * (otherSegment.controlPoints[0].x + otherSegment.controlPoints[1].x), 0.5f * (otherSegment.controlPoints[0].y + otherSegment.controlPoints[1].y)});
					subdividedSegment[0].controlPoints.push_back({0.25f * (otherSegment.controlPoints[0].x + otherSegment.controlPoints[2].x) + 0.5 * otherSegment.controlPoints[1].x, 0.25f * (otherSegment.controlPoints[0].y + otherSegment.controlPoints[2].y) + 0.5 * otherSegment.controlPoints[1].y});
					
					otherPathSegments.push_back(subdividedSegment[0]);
					ResolveQuadraticQuadraticOverlap(iteratedPathSegments, otherPathSegments, minTriangleArea);
					
					subdividedSegment[1].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[1].controlPoints.push_back(subdividedSegment[0].controlPoints[2]);
					subdividedSegment[1].controlPoints.push_back({0.5f * (otherSegment.controlPoints[1].x + otherSegment.controlPoints[2].x), 0.5f * (otherSegment.controlPoints[1].y + otherSegment.controlPoints[2].y)});
					subdividedSegment[1].controlPoints.push_back(otherSegment.controlPoints[2]);
					
					otherPathSegments.push_back(subdividedSegment[1]);
					ResolveQuadraticQuadraticOverlap(iteratedPathSegments, otherPathSegments, minTriangleArea);

					otherSegment = otherPathSegments.back();
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
					ResolveQuadraticQuadraticOverlap(otherPathSegments, newSegments0, minTriangleArea);
					iteratedPathSegments.insert(iteratedPathSegments.begin() + i, newSegments0.begin(), newSegments0.end());
					i += newSegments0.size();
					
					subdividedSegment[1].type = PathSegment::TypeBezierQuadratic;
					subdividedSegment[1].controlPoints.push_back(subdividedSegment[0].controlPoints[2]);
					subdividedSegment[1].controlPoints.push_back({0.5f * (iteratedSegment.controlPoints[1].x + iteratedSegment.controlPoints[2].x), 0.5f * (iteratedSegment.controlPoints[1].y + iteratedSegment.controlPoints[2].y)});
					subdividedSegment[1].controlPoints.push_back(iteratedSegment.controlPoints[2]);
					
					std::vector<PathSegment> newSegments1;
					newSegments1.push_back(subdividedSegment[1]);
					ResolveQuadraticQuadraticOverlap(otherPathSegments, newSegments1, minTriangleArea);
					iteratedPathSegments.insert(iteratedPathSegments.begin() + i, newSegments1.begin(), newSegments1.end());
					i += newSegments1.size() - 1;
				}
			}
		}
	}

	const PathCollection MeshGeneratorLoopBlinn::ResolveOverlaps(const PathCollection &paths, double minTriangleArea)
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
								ResolveQuadraticQuadraticOverlap(newSegments, oldSegments, minTriangleArea);
								
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
	const TriangleMesh MeshGeneratorLoopBlinn::GetMeshForPathCollection(const PathCollection &paths)
	{
		PathCollection filteredPaths = DowngradeCubicSegments(paths);
		filteredPaths = FilterDegenerateSegments(filteredPaths, 0.1);
		filteredPaths = ResolveIntersections(filteredPaths);
		filteredPaths = ResolveOverlaps(filteredPaths);
		filteredPaths = FindWindingOrder(filteredPaths);
		
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
					isNotFirstSegment = true;
				}
				
				if(segment.type == PathSegment::TypeLine)
				{
					outline.points.push_back(segment.controlPoints[1]);
				}
				else if(segment.type == PathSegment::TypeBezierQuadratic)
				{
					int8_t onLineResult = Math::IsOnLine(segment.controlPoints[0], segment.controlPoints[2], segment.controlPoints[1]);
					int8_t direction = 0;
					if(segment.isFilledOutside)
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
						//TODO: Support rational quadratic curves to be able to exactly model an arc for example? This is somewhat explained in the Loop Blinn paper: https://www.microsoft.com/en-us/research/wp-content/uploads/2005/01/p1000-loop.pdf section 3.2
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

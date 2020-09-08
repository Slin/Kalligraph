//
//  KGCommon.cpp
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#include "KGCommon.h"
#include <cmath>

namespace KG
{
	double Vector2::GetDotProduct(const Vector2 &other) const
	{
		return x * other.x + y * other.y;
	}

	float Vector2::GetCrossProduct(const Vector2 &other) const
	{
		return x * other.y - y * other.x;
	}
		
	Vector3 Vector2::GetCrossProductSimplified(const Vector2 &other) const
	{
		Vector3 result;
		result.x = y - other.y;
		result.y = other.x - x;
		result.z = x*other.y - y*other.x;
		return result;
	}
		

	double Vector2::GetDotProductSimplified(const Vector3 &other) const
	{
		return x * other.x + y * other.y + other.z;
	}

	Vector3 Vector3::GetCrossProduct(const Vector3 &other) const
	{
		Vector3 result;
		result.x = y*other.z - z*other.y;
		result.y = z*other.x - x*other.z;
		result.z = x*other.y - y*other.x;
		return result;
	}
	
	double Vector3::GetDotProduct(const Vector3 &other) const
	{
		return x * other.x + y * other.y + z * other.z;
	}
	
	double Vector3::GetDotProductSimplified(const Vector2 &other) const
	{
		return x * other.x + y * other.y + z;
	}


	bool Math::IsCCW(const Vector2 &A, const Vector2 &B, const Vector2 &C)
	{
		//Based on https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
		
		//Returns true if ABC are a triangle with counter clockwise winding order like this:
		//  C
		// / \
		//A---B
		
		//False for everything else (collinear or clockwise)
		
		return (B.x-A.x) * (C.y-A.y) > (B.y-A.y) * (C.x-A.x);
	}

	bool Math::AreLineSegmentsIntersecting(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D)
	{
		//Based on https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
		
		//Two lines are intersecting if the points of one line are on opposite sides of the other line (second check)
		//AND as this is handling line segments, the first check verifies that they are crossing within the segments
		return IsCCW(A,C,D) != IsCCW(B,C,D) && IsCCW(A,B,C) != IsCCW(A,B,D);
	}

	bool Math::AreTrianglesIntersecting(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E, const Vector2 &F)
	{
		//I found the idea of finding the separating axis in some comment online, and remembered this one having a good description: https://www.toptal.com/game/video-game-physics-part-ii-collision-detection-for-solid-objects
		
		//If either side of a triangle has all points of the other triangle on the outside, they do not overlap
		//If this isn't true for any side, they do overlap
		
		if(!IsCCW(A, B, C))
		{
			if(IsOnLine(C, A, D) >= 0 && IsOnLine(C, A, E) >= 0 && IsOnLine(C, A, F) >= 0) return false; //This one is most likely to be the separating axis
			if(IsOnLine(A, B, D) >= 0 && IsOnLine(A, B, E) >= 0 && IsOnLine(A, B, F) >= 0) return false;
			if(IsOnLine(B, C, D) >= 0 && IsOnLine(B, C, E) >= 0 && IsOnLine(B, C, F) >= 0) return false;
		}
		else
		{
			if(IsOnLine(C, A, D) <= 0 && IsOnLine(C, A, E) <= 0 && IsOnLine(C, A, F) <= 0) return false; //This one is most likely to be the separating axis
			if(IsOnLine(A, B, D) <= 0 && IsOnLine(A, B, E) <= 0 && IsOnLine(A, B, F) <= 0) return false;
			if(IsOnLine(B, C, D) <= 0 && IsOnLine(B, C, E) <= 0 && IsOnLine(B, C, F) <= 0) return false;
		}
		
		if(!IsCCW(D, E, F))
		{
			if(IsOnLine(F, D, A) >= 0 && IsOnLine(F, D, B) >= 0 && IsOnLine(F, D, C) >= 0) return false; //This one is most likely to be the separating axis
			if(IsOnLine(D, E, A) >= 0 && IsOnLine(D, E, B) >= 0 && IsOnLine(D, E, C) >= 0) return false;
			if(IsOnLine(E, F, A) >= 0 && IsOnLine(E, F, B) >= 0 && IsOnLine(E, F, C) >= 0) return false;
		}
		else
		{
			if(IsOnLine(F, D, A) <= 0 && IsOnLine(F, D, B) <= 0 && IsOnLine(F, D, C) <= 0) return false; //This one is most likely to be the separating axis
			if(IsOnLine(D, E, A) <= 0 && IsOnLine(D, E, B) <= 0 && IsOnLine(D, E, C) <= 0) return false;
			if(IsOnLine(E, F, A) <= 0 && IsOnLine(E, F, B) <= 0 && IsOnLine(E, F, C) <= 0) return false;
		}
		
		return true;
	}

	float Math::GetSquaredTriangleArea(const Vector2 &A, const Vector2 &B, const Vector2 &C)
	{
		//Based on https://en.wikipedia.org/wiki/Heron%27s_formula
		
		//Get squared length of each side of the triangle
		Vector2 AB = {A.x - B.x, A.y - B.y};
		Vector2 BC = {B.x - C.x, B.y - C.y};
		Vector2 CA = {C.x - A.x, C.y - A.y};
		float a2 = AB.GetDotProduct(AB);
		float b2 = BC.GetDotProduct(BC);
		float c2 = CA.GetDotProduct(CA);
		
		//Use herons formula to get the squared triangle area
		float result = 2.0f*a2*b2 + 2.0f*a2*c2 + 2.0f*b2*c2 - a2*a2 - b2*b2 - c2*c2;
		return 0.125 * result;
	}

	int8_t Math::IsOnLine(const Vector2 &A, const Vector2 &B, const Vector2 &C, double epsilon)
	{
		//Same as IsCCW, but with an epsilon to add some tolerance
		//returns 0 if C is on AC, 1 if it's CCW, -1 if it is CW
		double result = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
		if(std::abs(result) < epsilon) return 0;
		if(result < 0) return -1;
		return 1;
	}

	Vector2 Math::GetIntersectionPoint(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D)
	{
		//Based on https://stackoverflow.com/a/20677983
		
		double line1[3];
		line1[0] = A.y - B.y;
		line1[1] = B.x - A.x;
		line1[2] = -A.x * B.y + B.x * A.y;
		
		double line2[3];
		line2[0] = C.y - D.y;
		line2[1] = D.x - C.x;
		line2[2] = -C.x * D.y + D.x * C.y;
		
		double d = line1[0] * line2[1] - line1[1] * line2[0];
		if(d == 0) return Vector2{0.0f, 0.0f}; //Prevent dividing with 0, this function should only be called if there is a known intersection, so this shouldn't ever happen...
		
		double dx = line1[2] * line2[1] - line1[1] * line2[2];
		double dy = line1[0] * line2[2] - line1[2] * line2[0];
		
		return Vector2{dx/d, dy/d};
	}

	bool Math::IsQuadraticCurveIntersectingLineSegment(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E)
	{
		//Based on the same idea as the line - line intersection test, just checking which side of a quadratic curve something is on is a bit more involved.
		
		//Check that if they do intersect, the intersection is within the limits of the curve and the line segment, if not return early.
		if(IsCCW(A,D,E) == IsCCW(C,D,E)) return false;
		
		//Calculate baryzentric coordinates for line control points (https://en.wikipedia.org/wiki/Barycentric_coordinate_system)
		Vector2 BA = {B.x - A.x, B.y - A.y};
		Vector2 CA = {C.x - A.x, C.y - A.y};
		double den = BA.x * CA.y - CA.x * BA.y;
		
		if(den == 0) return false; //Prevent division by 0, in this case ABC are collinear, should probably fall back to line - line intersection in this case...
		
		Vector2 DA = {D.x - A.x, D.y - A.y};
		double vDA = (DA.x * CA.y - CA.x * DA.y) / den;
		double wDA = (BA.x * DA.y - DA.x * BA.y) / den;
		
		Vector2 EA = {E.x - A.x, E.y - A.y};
		double vEA = (EA.x * CA.y - CA.x * EA.y) / den;
		double wEA = (BA.x * EA.y - EA.x * BA.y) / den;
		
		//Quadratic curve equation, 0 if D is ON the curve, < or > depending on what side it is on
		Vector2 uvD;
		uvD.x = 0.5 * vDA + wDA;
		uvD.y = wDA;
		double sideD = uvD.x*uvD.x - uvD.y;
		
		//Quadratic curve equation, 0 if E is ON the curve, < or > depending on what side it is on
		Vector2 uvE;
		uvE.x = 0.5 * vEA + wEA;
		uvE.y = wEA;
		double sideE = uvE.x*uvE.x - uvE.y;
		
		//There is an intersection if D and E are on different sides of the quadratic curve.
		return (sideD > 0) - (sideD < 0) != (sideE > 0) - (sideE < 0);
	}

	std::vector<double> Math::GetQuadraticCurveAndLineSegmentIntersectionCoefficients(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E)
	{
		//Based on https://stackoverflow.com/a/50135437
		
		std::vector<double> result;
		
		// solving for t
		Vector2 lineDir = {E.x - D.x, E.y - D.y};
		Vector2 alpha = {A.x + C.x - 2.0 * B.x, A.y + C.y - 2.0 * B.y};
		Vector2 beta = {B.x - A.x, B.y - A.y};
		Vector2 gamma = {A.x - D.x, A.y - D.y};
		double a = alpha.GetCrossProduct(lineDir);
		double b = beta.GetCrossProduct(lineDir) * 2.0;
		double c = gamma.GetCrossProduct(lineDir);

		double t[2] = {-1.0, -1.0};
		
		double d = b * b - 4.0 * a * c;
		if(d < 0.0) //no intersection at all
		{
			return result;
		}
		else if(d > 0.0) //two possible intersections
		{
			double sd = std::sqrt(d);
			t[0] = (-b - sd) / (2.0 * a);
			t[1] = (-b + sd) / (2.0 * a);
		}
		else //one possible intersection
		{
			t[0] = -b / (2.0 * a);
		}
		
		bool useX = std::abs(lineDir.x) > std::abs(lineDir.y);
		for(int i = 0; i < 2; i++)
		{
			//== 0 and == 1 would technically also be intersections, but the way this is used, I am not interested in those
			if(t[i] <= 0.0 || t[i] >= 1.0) continue;
			
			//find s and check if is within line range
			double u = (1 - t[i]) * (1 - t[i]);
			double v = 2 * t[i] * (1 - t[i]);
			double w = t[i] * t[i];
			double s = useX? ((u * A.x + v * B.x + w * C.x - D.x) / lineDir.x) : ((u * A.y + v * B.y + w * C.y - D.y) / lineDir.y);
			if(s <= 0.0 || s >= 1.0) continue;

			result.push_back(t[i]);
		}
		
		return result;
	}

	std::vector<double> Math::GetQuadraticCurveAndQuadraticCurveIntersectionCoefficients(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E, const Vector2 &F)
	{
		//Based on https://stackoverflow.com/a/50142103
		
		std::vector<double> result;
		
		Vector2 P0 = A;
		Vector2 P1 = {B.x - A.x, B.y - A.y};
		Vector2 P2 = {A.x - 2.0 * B.x + C.x, A.y - 2.0 * B.y + C.y};
		
		//Calculate baryzentric coordinates for DEF inside P0P1P2 (https://en.wikipedia.org/wiki/Barycentric_coordinate_system)
		Vector2 BA = {P1.x - P0.x, P1.y - P0.y};
		Vector2 CA = {P2.x - P0.x, P2.y - P0.y};
		double den = BA.x * CA.y - CA.x * BA.y;
		
		if(den == 0.0) return result; //Prevent division by 0, happens if P0, P1 and P2 are collinear should probably fall back to curve - line intersection?
		
		Vector2 DA = {D.x - P0.x, D.y - P0.y};
		double vDA = (DA.x * CA.y - CA.x * DA.y) / den;
		double wDA = (BA.x * DA.y - DA.x * BA.y) / den;
		
		Vector2 EA = {E.x - P0.x, E.y - P0.y};
		double vEA = (EA.x * CA.y - CA.x * EA.y) / den;
		double wEA = (BA.x * EA.y - EA.x * BA.y) / den;
		
		Vector2 FA = {F.x - P0.x, F.y - P0.y};
		double vFA = (FA.x * CA.y - CA.x * FA.y) / den;
		double wFA = (BA.x * FA.y - FA.x * BA.y) / den;
		
		double fx0 = (vDA - 2.0 * vEA + vFA);
		double fx1 = (-2.0 * vDA + 2.0 * vEA);
		double fx2 = vDA;

		double fy0 = 4.0 * (wDA - 2.0 * wEA + wFA);
		double fy1 = 4.0 * (-2.0 * wDA + 2.0 * wEA);
		double fy2 = 4.0 * wDA;
		
		double gx0 = fx0 * fx0;
		double gx1 = 2.0 * fx0 * fx1;
		double gx2 = (2.0 * fx0 * fx2 + fx1 * fx1);
		double gx3 = 2.0 * fx1 * fx2;
		double gx4 = fx2 * fx2;
		
		double h0 = (gx2 - fy0);
		double h1 = (gx3 - fy1);
		double h2 = (gx4 - fy2);
		
		//X = vDA * (1-t)^2 + 2 * vEA * (1-t) * t + vFA * t^2
		//Y = wDA * (1-t)^2 + 2 * wEA * (1-t) * t + wFA * t^2
		//X^2 = 4*Y
		//These equations end up in
		//gx0 * t^4 + gx1 * t^3 + h0 * t^2 - h1 * t - h2 = 0
		//which is quartic equation that can be solved
		//The following is based on https://github.com/sidneycadot/quartic/blob/master/solve-quartic.cc
		double a = gx0;
		double b = gx1 / a;
		double c = h0 / a;
		double d = h1 / a;
		double e = h2 / a;

		double Q1 = c * c - 3.0 * b * d + 12.0 * e;
		double Q2 = 2.0 * c * c * c - 9.0 * b * c * d + 27.0 * d * d + 27.0 * b * b * e - 72.0 * c * e;
		double Q3 = 8.0 * b * c - 16.0 * d - 2.0 * b * b * b;
		double Q4 = 3.0 * b * b - 8.0 * c;

		double Q5Root = Q2 * Q2 / 4.0 - Q1 * Q1 * Q1;
		if(Q5Root < 0.0)
		{
			return result;
		}
		
		double Q5 = std::cbrt(Q2 / 2.0 + std::sqrt(Q5Root));
		double Q6 = (Q1 / Q5 + Q5) / 3.0;
		double Q7 = 2.0 * std::sqrt(Q4 / 12.0 + Q6);
		
		double den1 = 4.0 * Q4 / 6.0 - 4.0 * Q6 - Q3 / Q7;
		double den2 = 4.0 * Q4 / 6.0 - 4.0 * Q6 + Q3 / Q7;

		if(den1 == 0.0)
		{
			double solution = (-b - Q7) / 4.0;
			if(solution > 0.0 && solution < 1.0)
			{
				result.push_back(solution);
			}
		}
		if(den1 > 0.0)
		{
			double solution1 = (-b - Q7 - std::sqrt(den1)) / 4.0;
			if(solution1 > 0.0 && solution1 < 1.0)
			{
				result.push_back(solution1);
			}
			
			double solution2 = (-b - Q7 + std::sqrt(den1)) / 4.0;
			if(solution2 > 0.0 && solution2 < 1.0)
			{
				result.push_back(solution2);
			}
		}
		
		if(den2 == 0.0)
		{
			double solution = (-b + Q7) / 4.0;
			if(solution > 0.0 && solution < 1.0)
			{
				result.push_back(solution);
			}
		}
		if(den2 > 0.0)
		{
			double solution1 = (-b - Q7 - std::sqrt(den2)) / 4.0;
			if(solution1 > 0.0 && solution1 < 1.0)
			{
				result.push_back(solution1);
			}
			
			double solution2 = (-b - Q7 + std::sqrt(den2)) / 4.0;
			if(solution2 > 0.0 && solution2 < 1.0)
			{
				result.push_back(solution2);
			}
		}
		
		//Check if the results are also on DEF
		for(int i = 0; i < result.size(); i++)
		{
			double t = result[i];
			
			//Get intersection position
			Vector2 pos;
			pos.x = (1.0 - t) * (1.0 - t) * A.x + (1.0 - t) * t * B.x + t * t * C.x;
			pos.y = (1.0 - t) * (1.0 - t) * A.y + (1.0 - t) * t * B.y + t * t * C.y;
			
			//Insert position into quadratic equation and solve for the interpolation factor (o1 and o2 as there are two possible results)
			//x^2 + px + q = 0
			double l = (D.x - 2.0 * E.x + F.x); //It's not a quadratic curve if l == 0, should probably check though...
			double p = (-2.0 * D.x + 2.0 * E.x) / l;
			double q = D.x / l;
			
			double sq = 0.25 * p * p - q;
			if(sq < 0.0)
			{
				result.erase(result.begin() + i);
				i -= 1;
				continue;
			}
			
			//Calculate o1 and check if it's a valid point.
			double root = std::sqrt(sq);
			double o1 = - 0.5 * p + root;
			if(o1 > 0.0 && o1 < 0.0)
			{
				double ypos = (1.0 - o1) * (1.0 - o1) * D.y + (1.0 - o1) * o1 * E.y + o1 * o1 * F.y;
				if(std::abs(pos.y - ypos) < std::numeric_limits<double>::epsilon())
				{
					//If it's good, insert into result and stop here.
					result.insert(result.begin() + i + 1, o1);
					i += 1;
					continue;
				}
			}
			
			//Claculate o2 and check if it's a valid point.
			double o2 = - 0.5 * p - root;
			if(o2 > 0.0 && o2 < 0.0)
			{
				double ypos = (1.0 - o2) * (1.0 - o2) * D.y + (1.0 - o2) * o2 * E.y + o2 * o2 * F.y;
				if(std::abs(pos.y - ypos) < std::numeric_limits<double>::epsilon())
				{
					//If it's good, insert into result and stop here.
					result.insert(result.begin() + i + 1, o2);
					i += 1;
					continue;
				}
			}
			
			//Remove from result if both checks before failed.
			result.erase(result.begin() + i);
			i -= 1;
		}
		
		return result;
		
		
/*		x = gx0 * t^4 + gx1 * t^3 + gx2 * t^2 + gx3 * t + gx4;
		y = fy0 * t^2 + fy1 * t + fy2
		
		gx0 * t^4 + gx1 * t^3 + gx2 * t^2 + gx3 * t + gx4 = fy0 * t^2 + fy1 * t + fy2
		gx0 * t^4 + gx1 * t^3 + gx2 * t^2 + gx3 * t + gx4 - fy0 * t^2 - fy1 * t - fy2 = 0
		*/
		
/*		X^2 = 4*Y
		X = vDA * (1-t)^2 + 2 * vEA * (1-t) * t + vFA * t^2
		Y = wDA * (1-t)^2 + 2 * wEA * (1-t) * t + wFA * t^2
 
 		X = vDA - 2 * vDA * t + vDA * t * t + 2 * vEA * t - 2 * vEA * t * t + vFA * t * t
 		X = (vDA - 2 * vEA + vFA) * (t * t) + (-2 * vDA + 2 * vEA) * t + vDA
 
 		X = fx0 * (t * t) + fx1 * t + fx2
 		Y = fy0 * (t * t) + fy1 * t + fy2
 		X^2 = Y
 
 		X^2 = (fx0 * t^2 + fx1 * t + fx2) * (fx0 * t^2 + fx1 * t + fx2)
 		X^2 = fx2 * fx2 + 2.0 * fx2 * fx1 * t + fx1 * fx1 * t^2 + 2.0 * fx0 * fx2 * t^2 + 2.0 * fx0 * fx1 * t^3 + fx0 * fx0 * t^4
 		X^2 = (fx0 * fx0) * t^4 + (2.0 * fx0 * fx1) * t^3 + (2.0 * fx0 * fx2 + fx1 * fx1) * t^2 + (2.0 * fx1 * fx2) * t + (fx2 * fx2)
 
 		gx0 * t^4 + gx1 * t^3 + gx2 * t^2 + gx3 * t + gx4 = fy0 * t^2 + fy1 * t + fy2
 		gx0 * t^4 + gx1 * t^3 + gx2 * t^2 + gx3 * t + gx4 - fy0 * t^2 - fy1 * t - fy2 = 0
 		gx0 * t^4 + gx1 * t^3 + (gx2 - fy0) * t^2 + (gx3 - fy1) * t + (gx4 - fy2) = 0
 		
 		gx0 * t^4 + gx1 * t^3 + h0 * t^2 + h1 * t + h2 = 0
 
 
 		double fx0 = (vDA - 2.0 * vEA + vFA);
		double fx1 = (-2.0 * vDA + 2.0 * vEA);
		double fx2 = vDA;
 
 		double fy0 = 4.0 * (wDA - 2.0 * wEA + wFA);
		double fy1 = 4.0 * (-2.0 * wDA + 2.0 * wEA);
		double fy2 = 4.0 * wDA;
 
		double gx0 = fx0 * fx0;
		double gx1 = 2.0 * fx0 * fx1;
		double gx2 = (2.0 * fx0 * fx2 + fx1 * fx1);
		double gx3 = 2.0 * fx1 * fx2;
		double gx4 = fx2 * fx2;
 
 		double h0 = (gx2 - fy0);
		double h1 = (gx3 - fy1);
		double h2 = (gx4 - fy2);
		
		(vDA * (1-t)^2 + 2 * vEA * (1-t) * t + vFA * t^2) * (vDA * (1-t)^2 + 2 * vEA * (1-t) * t + vFA * t^2)*/
	}
}

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

	bool Math::IsPointInTriangle(const Vector2 &point, const Vector2 &A, const Vector2 &B, const Vector2 &C)
	{
		if(IsCCW(A, B, C))
		{
			return (IsOnLine(A, B, point) >= 0 && IsOnLine(B, C, point) >= 0 && IsOnLine(C, A, point) >= 0);
		}
		else
		{
			return (IsOnLine(A, B, point) <= 0 && IsOnLine(B, C, point) <= 0 && IsOnLine(C, A, point) <= 0);
		}
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

	double Math::GetSquaredTriangleArea(const Vector2 &A, const Vector2 &B, const Vector2 &C)
	{
		//Based on https://en.wikipedia.org/wiki/Heron%27s_formula
		
		//Get squared length of each side of the triangle
		Vector2 AB = {A.x - B.x, A.y - B.y};
		Vector2 BC = {B.x - C.x, B.y - C.y};
		Vector2 CA = {C.x - A.x, C.y - A.y};
		double a2 = AB.GetDotProduct(AB);
		double b2 = BC.GetDotProduct(BC);
		double c2 = CA.GetDotProduct(CA);
		
		//Use herons formula to get the squared triangle area
		double result = 2.0*a2*b2 + 2.0*a2*c2 + 2.0*b2*c2 - a2*a2 - b2*b2 - c2*c2;
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
		double a = alpha.GetCrossProduct(lineDir);
		if(std::abs(a) < std::numeric_limits<double>::epsilon()) //No intersection if a is 0
		{
			return result;
		}
		
		Vector2 beta = {B.x - A.x, B.y - A.y};
		Vector2 gamma = {A.x - D.x, A.y - D.y};
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
			double results[2];
			results[0] = (-b - sd) / (2.0 * a);
			results[1] = (-b + sd) / (2.0 * a);
			if(results[0] < results[1])
			{
				t[0] = results[0];
				t[1] = results[1];
			}
			else
			{
				t[0] = results[1];
				t[1] = results[0];
			}
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

	static std::complex<double> complex_sqrt(const std::complex<double> & z)
	{
		return pow(z, 1. / 2.);
	}

	static std::complex<double> complex_cbrt(const std::complex<double> & z)
	{
		return pow(z, 1. / 3.);
	}

	std::vector<double> Math::SolveQuarticEquation(std::complex<double> b, std::complex<double> c, std::complex<double> d, std::complex<double> e)
	{
		//Find x for x^4 + b * x^3 + c * x^2 + d * x + e = 0
		//This is taken from https://github.com/sidneycadot/quartic/blob/master/solve-quartic.cc
		std::vector<double> result;

		const std::complex<double> Q1 = c * c - 3. * b * d + 12. * e;
		const std::complex<double> Q2 = 2. * c * c * c - 9. * b * c * d + 27. * d * d + 27. * b * b * e - 72. * c * e;
		const std::complex<double> Q3 = 8. * b * c - 16. * d - 2. * b * b * b;
		const std::complex<double> Q4 = 3. * b * b - 8. * c;

		const std::complex<double> Q5 = complex_cbrt(Q2 / 2. + complex_sqrt(Q2 * Q2 / 4. - Q1 * Q1 * Q1));
		const std::complex<double> Q6 = (Q1 / Q5 + Q5) / 3.;
		const std::complex<double> Q7 = 2. * complex_sqrt(Q4 / 12. + Q6);

		std::complex<double> roots[4];
		roots[0] = (-b - Q7 - complex_sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)) / 4.;
		roots[1] = (-b - Q7 + complex_sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)) / 4.;
		roots[2] = (-b + Q7 - complex_sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)) / 4.;
		roots[3] = (-b + Q7 + complex_sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)) / 4.;
		
		for(int i = 0; i < 4; i++)
		{
			if(std::abs(roots[i].imag()) < std::numeric_limits<double>::epsilon())
			{
				if(roots[i].real() > 0.0000001 && roots[i].real() < 1.0 - 0.0000001)
				{
					result.push_back(roots[i].real());
				}
			}
		}
		
		return result;
	}

	std::vector<double> Math::GetQuadraticCurveAndQuadraticCurveIntersectionCoefficients(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E, const Vector2 &F)
	{
		//This one is my own, as besides some comments that it's trivial to do I didn't find much info about it.
		//Turns out that it's not hard, but it's easy to miss a term or accidentially flip a sign...
		
		std::vector<double> result;
		
		/*
		I got these two equations from the curves and am looking for t. The only other unknown is o.
		A.x * (1-t)^2 + 2 * B.x * (1-t) * t + C.x * t^2 = D.x * (1-o)^2 + 2 * E.x * (1-o) * o + F.x * o^2
		A.y * (1-t)^2 + 2 * B.y * (1-t) * t + C.y * t^2 = D.y * (1-o)^2 + 2 * E.y * (1-o) * o + F.y * o^2
		 
		Substituting factors in front of the unknowns helps to keep things shorter.
		 
		1. I am solving the first equation for o:
		(A.x - 2 * B.x + C.x) * t^2 + (-2 * A.x + 2 * B.x) * t + A.x = (D.x - 2 * E.x + F.x) * o^2 + (-2 * D.x + 2 * E.x) * o + D.x
		<=> (-D.x + 2 * E.x - F.x) * o^2 + (2 * D.x - 2 * E.x) * o + [(A.x - 2 * B.x + C.x) * t^2 + (-2 * A.x + 2 * B.x) * t + A.x - D.x] = 0
		after substitution
		fx0 * o^2 + fx1 * o + (fx2 * t^2 + fx3 * t + fx4) = 0
		=>
		o1 = -0.5 * fx1/fx0 + sqrt[0.25 * (fx1/fx0)^2 - fx2/fx0 * t^2 - fx3/fx0 * t - fx4/fx0]
		o2 = -0.5 * fx1/fx0 - sqrt[0.25 * (fx1/fx0)^2 - fx2/fx0 * t^2 - fx3/fx0 * t - fx4/fx0]
		can by simplified to
		o1 = gx0 + sqrt[gx1 - gx2 * t^2 - gx3 * t]
		o2 = gx0 - sqrt[gx1 - gx2 * t^2 - gx3 * t]
		 
		2. Replacing o in the second equation with the results from previous step:
		(A.y - 2 * B.y + C.y) * t^2 + (-2 * A.y + 2 * B.y) * t + (-D.y + 2 * E.y - F.y) * o^2 + (2 * D.y - 2 * E.y) * o + (A.y - D.y) = 0
		after substitution
		fy0 * t^2 + fy1 * t + fy2 * o^2 + fy3 * o + fy4 = 0
		insert solutions for o and move things around a bit, I did a couple steps on paper here to calculate the o^2+o part (the +- that is there due to the two solutions for o)
		(fy0 - fy2 * gx2) * t^2 + (fy1 - fy2 * gx3) * t + (fy3 * gx0 + fy2 * gx0^2 + fy2 * gx1 + fy4) + (fy3 + fy2 * gx0 * 2) * (+-sqrt[gx1 - gx2 * t^2 - gx3 * t]) = 0
		after substitution
		h0 * t^2 + h1 * t + h2 + h3 * (+-sqrt[gx1 - gx2 * t^2 - gx3 * t]) = 0
		<=> h0 * t^2 + h1 * t + h2 = -h3 * (-+sqrt[gx1 - gx2 * t^2 - gx3 * t])
		<=> (h0 * t^2 + h1 * t + h2)^2 = h3^2 * (gx1 - gx2 * t^2 - gx3 * t)
		<=> h2^2 + 2 * h1 * h2 * t + h1^2 * t^2 + 2 * h0 * h2 * t^2 + 2 * h0 * h1 * t^3 + h0^2 * t^4 = h3^2 * gx1 - h3^2 * gx2 * t^2 - h3^2 * gx3 * t
		<=> h2^2 + 2 * h1 * h2 * t + h1^2 * t^2 + 2 * h0 * h2 * t^2 + 2 * h0 * h1 * t^3 + h0^2 * t^4 - gx1 * h3^2 + gx3 * h3^2 * t + gx2 * h3^2 * t^2 = 0
		<=> (h0^2) * t^4 + (2 * h0 * h1) * t^3 + (h1^2 + 2 * h0 * h2 + gx2 * h3^2) * t^2 + (2 * h1 * h2 + gx3 * h3^2) * t + (h2^2 - gx1 * h3^2) = 0
		after substitution
		t^4 + m1 * t^3 + m2 * t^2 + m3 * t + m4 = 0
		 
		m1, m2, m3 and m4 are then just put into the quartic solver and it's done :)
		Subsitution is always with the variables below, replacing the parts in brackets.
		*/
		
		double fx0 = (-D.x + 2.0 * E.x - F.x);
		double fx1 = (2.0 * D.x - 2.0 * E.x);
		double fx2 = (A.x - 2.0 * B.x + C.x);
		double fx3 = (-2.0 * A.x + 2.0 * B.x);
		double fx4 = A.x - D.x;

		double fy0 = (A.y - 2.0 * B.y + C.y);
		double fy1 = (-2.0 * A.y + 2.0 * B.y);
		double fy2 = (-D.y + 2.0 * E.y - F.y);
		double fy3 = (2.0 * D.y - 2.0 * E.y);
		double fy4 = A.y - D.y;

		double gx0 = -0.5 * fx1/fx0;
		double gx1 = 0.25 * (fx1/fx0) * (fx1/fx0) - fx4/fx0;
		double gx2 = fx2/fx0;
		double gx3 = fx3/fx0;

		double h0 = (fy0 - fy2 * gx2);
		double h1 = (fy1 - fy2 * gx3);
		double h2 = fy3 * gx0 + fy2 * gx0 * gx0 + fy2 * gx1 + fy4;
		double h3 = (fy3 + fy2 * gx0 * 2.0);

		double m0 = h0 * h0;
		double m1 = (2.0 * h0 * h1) / m0;
		double m2 = (2.0 * h0 * h2 + h1 * h1 + gx2 * h3 * h3) / m0;
		double m3 = (2.0 * h1 * h2 + gx3 * h3 * h3) / m0;
		double m4 = (h2 * h2 - h3 * h3 * gx1) / m0;
		
		//Solve t^4 + m1 * t^3 + m2 * t^2 - m3 * t - m4 = 0 for t
		result = SolveQuarticEquation(m1, m2, m3, m4);
		
		//Check if the results are also on DEF
		for(int i = 0; i < result.size(); i++)
		{
			double t = result[i];
			
			//Get intersection position
			Vector2 pos;
			pos.x = (1.0 - t) * (1.0 - t) * A.x + 2.0 * (1.0 - t) * t * B.x + t * t * C.x;
			pos.y = (1.0 - t) * (1.0 - t) * A.y + 2.0 * (1.0 - t) * t * B.y + t * t * C.y;
			
			//Insert position into quadratic equation and solve for the interpolation factor (o1 and o2 as there are two possible results)
			//lo^2 + po + q = 0
			double l = (D.x - 2.0 * E.x + F.x); //It's not a quadratic curve if l == 0, should probably check though...
			double p = (-2.0 * D.x + 2.0 * E.x) / l;
			double q = (D.x - pos.x) / l;
			
			double sq = 0.25 * p * p - q;
			if(sq < 0.0)
			{
				result.erase(result.begin() + i);
				i -= 1;
				continue;
			}
			
			//Calculate o1 and check if it's a valid point.
			double root = std::sqrt(sq);
			double o1 = -0.5 * p + root;
			if(o1 > 0.0000001 && o1 < 1.0-0.0000001)
			{
				double ypos = (1.0 - o1) * (1.0 - o1) * D.y + 2.0 * (1.0 - o1) * o1 * E.y + o1 * o1 * F.y;
				if(std::abs(pos.y - ypos) < 0.0001)//std::numeric_limits<double>::epsilon())
				{
					//If it's good, insert into result and stop here.
					result.insert(result.begin() + i + 1, o1);
					i += 1;
					continue;
				}
			}
			
			//Claculate o2 and check if it's a valid point.
			double o2 = -0.5 * p - root;
			if(o2 > 0.0000001 && o2 < 1.0-0.0000001)
			{
				double ypos = (1.0 - o2) * (1.0 - o2) * D.y + 2.0 * (1.0 - o2) * o2 * E.y + o2 * o2 * F.y;
				if(std::abs(pos.y - ypos) < 0.0001)//std::numeric_limits<double>::epsilon())
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
	}
}

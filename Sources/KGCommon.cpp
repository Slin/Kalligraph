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
		//Returns true if ABC are a triangle with counter clockwise winding order like this:
		//  C
		// / \
		//A---B
		
		//False for everything else (collinear or clockwise)
		
		return (B.x-A.x) * (C.y-A.y) > (B.y-A.y) * (C.x-A.x);
	}

	bool Math::AreLineSegmentsIntersecting(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D)
	{
		//Two lines are intersecting if the points of one line are on opposite sides of the other line (second check)
		//AND as this is handling line segments, the first check verifies that they are crossing within the segments
		return IsCCW(A,C,D) != IsCCW(B,C,D) && IsCCW(A,B,C) != IsCCW(A,B,D);
	}

	bool Math::AreTrianglesIntersecting(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D, const Vector2 &E, const Vector2 &F)
	{
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
}

//
//  KGCommon.cpp
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#include "KGCommon.h"
#include <limits>
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
		return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
	}

	bool Math::IsIntersecting(const Vector2 &A, const Vector2 &B, const Vector2 &C, const Vector2 &D)
	{
		return IsCCW(A,C,D) != IsCCW(B,C,D) && IsCCW(A,B,C) != IsCCW(A,B,D);
	}

	int8_t Math::IsOnLine(const Vector2 &A, const Vector2 &B, const Vector2 &C)
	{
		double result = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
		if(std::abs(result) < std::numeric_limits<double>::epsilon()) return 0;
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

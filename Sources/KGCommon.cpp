//
//  KGCommon.cpp
//  Kalligraph
//
//  Copyright 2020 by Nils Daumann. All rights reserved.
//

#include "KGCommon.h"

namespace KG
{
	float Vector2::GetDotProduct(const Vector2 &other) const
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
		

	float Vector2::GetDotProductSimplified(const Vector3 &other) const
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
	
	float Vector3::GetDotProduct(const Vector3 &other) const
	{
		return x * other.x + y * other.y + z * other.z;
	}
	
	float Vector3::GetDotProductSimplified(const Vector2 &other) const
	{
		return x * other.x + y * other.y + z;
	}
}

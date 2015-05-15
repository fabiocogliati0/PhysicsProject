/*
* Progetto di Physics Programming
* Fabio Cogliati, Manuele Nerucci
*/

#include "Quaternion.h"

#pragma once

namespace Utils{

	class Vector3
	{
	public:
		// STATIC
		const static Vector3 zero;

		// VARS
		float x;
		float y;
		float z;

		// METHODS
		Vector3();
		Vector3(float px, float py, float pz);
		Vector3(const Vector3 &copy);
		Vector3& operator=(const Vector3 &right);
		Vector3 operator+(const Vector3 &right) const;
		Vector3& operator+=(const Vector3 &right);
		Vector3 operator-(const Vector3 &right) const;
		Vector3& operator-=(const Vector3 &right);
		Vector3 operator*(const float &right) const;
		Vector3 operator*(const Vector3 &right) const;
		Vector3& operator*=(const float &right);
		Vector3& operator*=(const Vector3 &right);
		Vector3 operator/(const float &right) const;
		Vector3& operator/=(const float &right);
		bool operator==(const Vector3 &right) const;
		bool operator!=(const Vector3 &right) const;
		float dot(const Vector3 &right) const; // dot product 
		Vector3 cross(const Vector3 &right) const; // cross product 
		void invert();  // invert of vector
		float module() const; // module of vector
		void normalize(); // normalize vector
		float distance(const Vector3 &right) const; // distance from two vector
		Quaternion toQuaternion() const; // from euler to quaternion
		~Vector3();
	};

}
#define _USE_MATH_DEFINES
#include <math.h>

#pragma once

namespace Utils
{
	class Vector3;
	class Matrix;
	
	class Quaternion
	{
		private:

		public:
			// VARS
			float s;
			float x;
			float y;
			float z;

			// METHOD
			Quaternion();
			Quaternion(float s, float x, float y, float z);
			Quaternion operator*(const Quaternion &right) const;
			Quaternion& operator*=(const Quaternion &right);
			void set(float s, float x, float y, float z);
			float module() const;
			void normalize();
			void toMatrix(Matrix &right) const;
			Vector3 toEuler() const;
	};
}
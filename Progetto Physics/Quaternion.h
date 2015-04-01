#include "Matrix.h"
#include <math.h>

#pragma once

namespace Utils
{
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
			Quaternion(float s, float x, float y, float z);
			Quaternion operator*(const Quaternion &right) const;
			Quaternion& operator*=(const Quaternion &right);
			float module() const;
			void normalize();
			void toMatrix(Matrix &right) const;
	};
}
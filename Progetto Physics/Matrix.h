#pragma once

#include "Vector3.h"

namespace Utils
{
	const size_t size = 9;
	
	class Matrix
	{
		private:
			float _Matrix[size];
			size_t _size = size;
		
		public:
			Matrix();
			float& operator[](unsigned int index);
			const float& operator[](unsigned int index) const;
			Vector3 RotateRelative(const Vector3 &input) const;
			Vector3 RotateAbsolute(const Vector3 &input) const;
			~Matrix();
	};
}
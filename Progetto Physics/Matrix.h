#pragma once

#include "Vector3.h"

namespace Utils
{
	const int size = 9;
	
	class Matrix
	{
		private:
			float _Matrix[size];
			unsigned int _size = size;
		
		public:
			Matrix();
			float& operator[](unsigned int index);
			Vector3 RotateRelative(Vector3 &input) const;
			Vector3 RotateAbsolute(Vector3 &input) const;
			~Matrix();
	};
}
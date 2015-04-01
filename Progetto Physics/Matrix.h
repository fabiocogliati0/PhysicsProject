#include "Vector3.h"

#pragma once

namespace Utils
{
	const int size = 9;
	
	class Matrix
	{
		private:
			float _Matrix[size];
			int _size = size;
		
		public:
			Matrix();
			float& operator[](int index);
			void RotateRelative(Vector3 &right) const;
			void RotateAbsolute(Vector3 &right) const;
			~Matrix();
	};
}
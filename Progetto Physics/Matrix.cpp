#include "Matrix.h"

namespace Utils
{
	Matrix::Matrix()
	{
		for (unsigned int i = 0; i < _size; i++)
			_Matrix[i] = 0;
	}

	float& Matrix::operator[](int index)
	{
		if (index >= _size)
			return _Matrix[0];
		else
			return _Matrix[index];
	}

	void Matrix::RotateRelative(Vector3 &right) const
	{
		right.x = _Matrix[0] * right.x + _Matrix[1] * right.y + _Matrix[2] * right.z;
		right.y = _Matrix[3] * right.x + _Matrix[4] * right.y + _Matrix[5] * right.z;
		right.z = _Matrix[6] * right.x + _Matrix[7] * right.y + _Matrix[8] * right.z;
	}
	
	void Matrix::RotateAbsolute(Vector3 &right) const
	{
		right.x = _Matrix[0] * right.x + _Matrix[3] * right.y + _Matrix[6] * right.z;
		right.y = _Matrix[1] * right.x + _Matrix[4] * right.y + _Matrix[7] * right.z;
		right.z = _Matrix[2] * right.x + _Matrix[5] * right.y + _Matrix[8] * right.z;
	}

	Matrix::~Matrix() {}
}
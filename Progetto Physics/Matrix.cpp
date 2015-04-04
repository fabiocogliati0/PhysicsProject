#include "Matrix.h"

namespace Utils
{
	Matrix::Matrix()
	{
		for (size_t i = 0; i < _size; ++i)
			_Matrix[i] = 0;
	}

	float& Matrix::operator[](size_t index)
	{
		if (index >= _size)
			return _Matrix[0];
		else
			return _Matrix[index];
	}

	const float& Matrix::operator[](size_t index) const
	{
		if (index >= _size)
			return _Matrix[0];
		else
			return _Matrix[index];
	}

	Vector3 Matrix::RotateRelative(const Vector3 &input) const
	{
		Vector3 tmp;
		tmp.x = _Matrix[0] * input.x + _Matrix[1] * input.y + _Matrix[2] * input.z;
		tmp.y = _Matrix[3] * input.x + _Matrix[4] * input.y + _Matrix[5] * input.z;
		tmp.z = _Matrix[6] * input.x + _Matrix[7] * input.y + _Matrix[8] * input.z;
		return tmp;
	}
	
	Vector3 Matrix::RotateAbsolute(const Vector3 &input) const
	{
		Vector3 tmp;
		tmp.x = _Matrix[0] * input.x + _Matrix[3] * input.y + _Matrix[6] * input.z;
		tmp.y = _Matrix[1] * input.x + _Matrix[4] * input.y + _Matrix[7] * input.z;
		tmp.z = _Matrix[2] * input.x + _Matrix[5] * input.y + _Matrix[8] * input.z;
		return tmp;
	}

	Matrix::~Matrix() {}
}
#include "Quaternion.h"
#include "Matrix.h"
#include "Vector3.h"

namespace Utils
{
	Quaternion::Quaternion() : Quaternion(1, 0, 0, 0)
	{
	}

	Quaternion::Quaternion(float s, float x, float y, float z)
	{
		this->s = s;
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Quaternion& Quaternion::operator*=(const Quaternion &right)
	{
		s = s * right.s - x * right.x - y * right.y - z * right.z;
		x = s * right.x + x * right.s + y * right.z - z * right.y;
		y = s * right.y - x * right.z + y * right.s + z * right.x;
		z = s * right.z + x * right.y - y * right.x + z * right.s;
		return *this;
	}

	Quaternion Quaternion::operator*(const Quaternion &right) const
	{
		Quaternion tmp = *this;
		return tmp *= right;
	}

	void Quaternion::set(float s, float x, float y, float z)
	{
		this->s = s;
		this->x = x;
		this->y = y;
		this->z = z;
	}

	float Quaternion::module() const
	{
		return sqrt((s * s) + (x * x) + (y * y) + (z * z));
	}

	void Quaternion::normalize()
	{
		float tmp = this->module();
		if (tmp > 0.000001f) 
		{
			s /= tmp;
			x /= tmp;
			y /= tmp;
			z /= tmp;
		}
	}

	Matrix Quaternion::toMatrix() const
	{
		Matrix tmp;

		tmp[0] = 1.0f - 2.0f * ((y * y) + (z * z));
		tmp[1] = 2.0f * ((x * y) + (s * z));
		tmp[2] = 2.0f * ((x * z) - (s * y));
		tmp[3] = 2.0f * ((x * y) - (s * z));
		tmp[4] = 1.0f - 2.0f * ((z * z) + (x * x));
		tmp[5] = 2.0f * ((y * z) + (s * x));
		tmp[6] = 2.0f * ((x * z) + (s * y));
		tmp[7] = 2.0f * ((y * z) - (s * x));
		tmp[8] = 1.0f - 2.0f * ((x * x) + (y * y));
		return tmp;
	}

	Vector3 Quaternion::toEuler() const
	{
		Vector3 tmp;

		float test = this->x * this->y + this->z * this->s;
		
		if (test > 0.499)
		{
			tmp.y = roundf(2 * atan2(this->x, this->s) * (180 / (static_cast<float>(M_PI))));
			tmp.z = roundf((static_cast<float>(M_PI)) / 2 * (180 / (static_cast<float>(M_PI))));
			tmp.x = 0;
			return tmp;
		}
		
		if (test < -0.499)
		{
			tmp.y = roundf(-2 * atan2(this->x, this->s) * (180 / (static_cast<float>(M_PI))));
			tmp.z = roundf(-(static_cast<float>(M_PI)) / 2 * (180 / (static_cast<float>(M_PI))));
			tmp.x = 0;
			return tmp;
		}
		
		float sqx = this->x * this->x;
		float sqy = this->y * this->y;
		float sqz = this->z * this->z;
		tmp.y = roundf(atan2(2 * this->y * this->s - 2 * this->x * this->z, 1 - 2 * sqy - 2 * sqz) * (180 / (static_cast<float>(M_PI))));
		tmp.z = roundf(asin(2 * test) * (180 / (static_cast<float>(M_PI))));
		tmp.x = roundf(atan2(2 * this->x * this->s - 2 * this->y * this->z, 1 - 2 * sqx - 2 * sqz) * (180 / (static_cast<float>(M_PI))));
		return tmp;
	}
}
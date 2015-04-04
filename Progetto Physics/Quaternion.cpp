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

	void Quaternion::toMatrix(Matrix &right) const
	{
		right[0] = 1.0f - 2.0f * ((y * y) + (z * z));
		right[1] = 2.0f * ((x * y) + (s * z));
		right[2] = 2.0f * ((x * z) - (s * y));
		right[3] = 2.0f * ((x * y) - (s * z));
		right[4] = 1.0f - 2.0f * ((z * z) + (x * x));
		right[5] = 2.0f * ((y * z) + (s * x));
		right[6] = 2.0f * ((x * z) + (s * y));
		right[7] = 2.0f * ((y * z) - (s * x));
		right[8] = 1.0f - 2.0f * ((x * x) + (y * y));
	}

	void Quaternion::toEuler(Vector3 &right) const
	{
		float test = this->x * this->y + this->z * this->s;
		
		if (test > 0.499)
		{
			right.y = roundf(2 * atan2(this->x, this->s) * (180 / M_PI));
			right.z = roundf(M_PI / 2 * (180 / M_PI));
			right.x = 0;
			return;
		}
		
		if (test < -0.499)
		{
			right.y = roundf(-2 * atan2(this->x, this->s) * (180 / M_PI));
			right.z = roundf(-M_PI / 2 * (180 / M_PI));
			right.x = 0;
			return;
		}
		
		float sqx = this->x * this->x;
		float sqy = this->y * this->y;
		float sqz = this->z * this->z;
		right.y = roundf(atan2(2 * this->y * this->s - 2 * this->x * this->z, 1 - 2 * sqy - 2 * sqz) * (180 / M_PI));
		right.z = roundf(asin(2 * test) * (180 / M_PI));
		right.x = roundf(atan2(2 * this->x * this->s - 2 * this->y * this->z, 1 - 2 * sqx - 2 * sqz) * (180 / M_PI));
	}
}
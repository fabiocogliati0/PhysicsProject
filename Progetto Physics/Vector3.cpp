#include "Vector3.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace Utils
{

	Vector3::Vector3()
	{
		x = 0;
		y = 0;
		z = 0;
	}

	Vector3::Vector3(float x, float y, float z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3::Vector3(const Vector3 &copy)
	{
		x = copy.x;
		y = copy.y;
		z = copy.z;
	}

	Vector3& Vector3::operator=(const Vector3 &right)
	{
		if (this != &right)
		{
			x = right.x;
			y = right.y;
			z = right.z;
		}
		return *this;
	}

	Vector3 Vector3::operator+(const Vector3 &right) const
	{
		Vector3 tmp = *this;
		tmp += right;
		return tmp;
	}

	Vector3& Vector3::operator+=(const Vector3 &right)
	{
		x = x + right.x;
		y = y + right.y;
		z = z + right.z;
		return *this;
	}

	Vector3 Vector3::operator-(const Vector3 &right) const
	{
		Vector3 tmp = *this;
		tmp -= right;
		return tmp;
	}

	Vector3& Vector3::operator-=(const Vector3 &right)
	{
		x = x - right.x;
		y = y - right.y;
		z = z - right.z;
		return *this;
	}

	Vector3 Vector3::operator*(const float &right) const
	{
		Vector3 tmp = *this;
		tmp *= right;
		return tmp;
	}

	Vector3& Vector3::operator*=(const float &right)
	{
		x = x * right;
		y = y * right;
		z = z * right;
		return *this;
	}

	Vector3 Vector3::operator/(const float &right) const
	{
		Vector3 tmp = *this;
		tmp /= right;
		return tmp;
	}

	Vector3& Vector3::operator/=(const float &right)
	{
		x = x / right;
		y = y / right;
		z = z / right;
		return *this;
	}

	bool Vector3::operator==(const Vector3 &right) const
	{
		return x == right.x && y == right.y && z == right.z;
	}

	bool Vector3::operator!=(const Vector3 &right) const
	{
		if (*this == right)
			return false;
		else
			return true;
	}

	float Vector3::dot(const Vector3 &right) const
	{
		return (x * right.x) + (y * right.y) + (z * right.z);
	}

	Vector3 Vector3::cross(const Vector3 &right) const
	{
		Vector3 tmp;
		tmp.x = (y * right.z) - (z * right.y);
		tmp.y = (z * right.x) - (x * right.z);
		tmp.z = (x * right.y) - (y * right.x);
		return tmp;
	}

	void Vector3::invert()
	{
		x = -x;
		y = -y;
		z = -z;
	}

	float Vector3::module() const
	{
		return sqrt((x * x) + (y * y) + (z * z)); 
	}

	void Vector3::normalize()
	{
		float module = this->module();
		if (module > 0.000001f)
		{
			x /= module;
			y /= module;
			z /= module;
		}
	}

	float Vector3::distance(Vector3 &right) const
	{
		Vector3 tmp = *this - right;
		return tmp.module();
	}

	Quaternion Vector3::toQuaternion() const
	{
		Quaternion tmp;
		float c1 = cos((this->y * (M_PI / 180)) / 2);
		float c2 = cos((this->z * (M_PI / 180)) / 2);
		float c3 = cos((this->x * (M_PI / 180)) / 2);
		float s1 = sin((this->y * (M_PI / 180)) / 2);
		float s2 = sin((this->z * (M_PI / 180)) / 2);
		float s3 = sin((this->x * (M_PI / 180)) / 2);
		tmp.s = (c1 * c2 * c3) - (s1 * s2 * s3);
		tmp.x = (s1 * s2 * c3) + (c1 * c2 * s3);
		tmp.y = (s1 * c2 * c3) + (c1 * s2 * s3);
		tmp.z = (c1 * s2 * c3) - (s1 * c2 * s3);
		return tmp;
	}

	Vector3::~Vector3()
	{
	}

	// STATIC INIT
	const Vector3 Vector3::zero(0, 0, 0);

}
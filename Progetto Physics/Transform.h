#pragma once

#include "Vector3.h"

namespace PhysicEngine
{
	struct Transform
	{
		Utils::Vector3 position;
		Utils::Matrix rotationMatrix;

		Vector3 getEulerRotation() const;

		void setEulerRotation(const Vector3& eulerAngles);

		Quaternion getQuaternionRotation() const;

		void setQuaternionRotation(const Quaternion& quaternionRotation);
	};
}
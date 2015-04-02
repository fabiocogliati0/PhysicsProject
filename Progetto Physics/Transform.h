#pragma once

#include "Vector3.h"
#include "Matrix.h"
#include "Quaternion.h"

namespace PhysicEngine
{
	struct Transform
	{
		Utils::Vector3 position;
		Utils::Matrix rotationMatrix;

		Utils::Vector3 getEulerRotation() const;

		void setEulerRotation(const Utils::Vector3& eulerAngles);

		Utils::Quaternion getQuaternionRotation() const;

		void setQuaternionRotation(const Utils::Quaternion& quaternionRotation);
	};
}
#pragma once

#include "Vector3.h"
#include "Matrix.h"
#include "Quaternion.h"

namespace PhysicEngine
{
	class Transform
	{
		public:
			Utils::Vector3 position;
			Utils::Matrix rotationMatrix;
			Utils::Quaternion quaternionMatrix;

			Utils::Vector3 getEulerRotation() const;
			void setEulerRotation(const Utils::Vector3& eulerAngles);
	};
}
/*
* Progetto di Physics Programming
* Fabio Cogliati, Manuele Nerucci
*/

#pragma once

#include "Vector3.h"
#include "Matrix.h"
#include "Quaternion.h"

namespace PhysicEngine
{
	class Transform
	{
		private:
			Utils::Vector3 position;
			Utils::Matrix rotationMatrix;
			Utils::Quaternion rotationQuaternion;

		public:
			const Utils::Vector3& getPosition() const;
			const Utils::Matrix& getRotationMatrix() const;
			const Utils::Quaternion& getRotationQuaternion() const;
			Utils::Vector3 getEulerRotation() const;
			
			void setPosition(const Utils::Vector3& newPosition);
			void setEulerRotation(const Utils::Vector3& eulerAngles);
			void setQuaternionRotation(const Utils::Quaternion& newQuaternion);
	};
}
/*
* Progetto di Physics Programming
* Fabio Cogliati, Manuele Nerucci
*/

#include "Transform.h"

namespace PhysicEngine
{
	const Utils::Vector3& Transform::getPosition() const
	{
		return position;
	}
	
	const Utils::Matrix& Transform::getRotationMatrix() const
	{
		return rotationMatrix;
	}
	
	const Utils::Quaternion& Transform::getRotationQuaternion() const
	{
		return rotationQuaternion;
	}
	
	Utils::Vector3 Transform::getEulerRotation() const
	{
		return rotationQuaternion.toEuler();
	}


	void Transform::setPosition(const Utils::Vector3& newPosition)
	{
		position = newPosition;
	}
	
	void Transform::setEulerRotation(const Utils::Vector3& eulerAngles)
	{
		rotationQuaternion = eulerAngles.toQuaternion();
		rotationMatrix = rotationQuaternion.toMatrix();
	}
	
	void Transform::setQuaternionRotation(const Utils::Quaternion& newQuaternion)
	{
		rotationQuaternion = newQuaternion;
		rotationMatrix = rotationQuaternion.toMatrix();
	}

};
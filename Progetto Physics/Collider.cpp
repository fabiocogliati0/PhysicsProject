#include "Collider.h"

#include "Vector3.h"

namespace PhysicEngine
{

	Collider::~Collider() {}

	const Utils::Vector3& Collider::getRawInertia() const
	{
		return rawInertia;
	}

	float Collider::getVolume() const
	{
		return volume;
	}

	void Collider::setRawInertia(const Utils::Vector3& rawInertia){
		this->rawInertia = rawInertia;
	}

	void Collider::setVolume(float volume)
	{
		this->volume = volume;
	}


}
#pragma once

#include "Vector3.h"

namespace PhysicEngine
{

	struct Collision
	{
		float deformation;
		Utils::Vector3 impactPoint;
		Utils::Vector3 normal;
		Utils::Vector3 impactSpeed;
	};

}
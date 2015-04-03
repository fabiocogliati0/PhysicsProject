#pragma once

#include "Collider.h"
#include "Vector3.h"

#include <vector>

namespace PhysicEngine
{

	struct Collision;
	class RigidBody;

	class PlaneCollider : public Collider
	{

	public:

		
		enum lookDirections
		{
			MajorLookDirection = 0,
			MinorLookDirection
		};

		PlaneCollider();

		PlaneCollider(float A, float B, float C, float D, lookDirections lookDirection);

		PlaneCollider* clone() const;

		ColliderType getColliderType() const;

		float getAFunctionCoefficient() const;

		float getBFunctionCoefficient() const;

		float getCFunctionCoefficient() const;

		float getDFunctionCoefficient() const;

		lookDirections getLookingDirection() const;


	private:

		float A;

		float B;

		float C;

		float D;

		lookDirections lookDirection;

	};

}
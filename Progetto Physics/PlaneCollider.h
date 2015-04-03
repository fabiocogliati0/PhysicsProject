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

		enum coordinate
		{
			X_coordinate = 0,
			Y_coordinate,
			Z_coordinate
		};
		
		enum lookDirections
		{
			PositiveLookDirection = 0,
			NegativeLookDirection
		};

		PlaneCollider();

		PlaneCollider(coordinate axis, float axisValue, lookDirections lookDirection);

		PlaneCollider* clone() const;

		ColliderType getColliderType() const;

		coordinate getAxis() const;

		float getAxisValue() const;

		lookDirections getLookingDirection() const;


	private:

		coordinate axis;
		float axisValue;
		lookDirections lookDirection;

	};

}
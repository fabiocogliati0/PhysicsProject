#include "PlaneCollider.h"

#include "Collider.h"
#include "Collision.h"
#include "Vector3.h"

#include <vector>

namespace PhysicEngine{

	PlaneCollider::PlaneCollider() 
		: axis(Y_coordinate), axisValue(0.0f), lookDirection(PlaneCollider::PositiveLookDirection)
	{}

	PlaneCollider::PlaneCollider(coordinate axis, float axisValue, lookDirections lookDirection)
		: axis(axis), axisValue(axisValue), lookDirection(lookDirection)
	{}

	PlaneCollider* PlaneCollider::clone() const
	{
		return new PlaneCollider(*this);
	}

	Collider::ColliderType PlaneCollider::getColliderType() const
	{
		return PlaneColliderType;
	}

	PlaneCollider::coordinate PlaneCollider::getAxis() const
	{
		return axis;
	}

	float PlaneCollider::getAxisValue() const
	{
		return axisValue;
	}

	PlaneCollider::lookDirections PlaneCollider::getLookingDirection() const
	{
		return lookDirection;
	}

}
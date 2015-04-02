#include "PlaneCollider.h"

#include "BoxCollider.h"
#include "SphereCollider.h"
#include "IntersectOperations.h"
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

	bool PlaneCollider::intersect(const RigidBody& i_rigidBody,
		const Collider& i_colliderOther,
		const RigidBody& i_rigidBodyOther,
		std::vector<Collision>& o_collisions
		)	const
	{
		return i_colliderOther.intersectWho(i_rigidBody, *this, i_rigidBodyOther, o_collisions);
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




	bool PlaneCollider::intersectWho(const RigidBody& i_rigidBody,
		const BoxCollider& i_colliderOther,
		const RigidBody& i_rigidBodyOther,
		std::vector<Collision>& o_collisions
		)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
	}

	bool PlaneCollider::intersectWho(const RigidBody& i_rigidBody,
		const SphereCollider& i_colliderOther,
		const RigidBody& i_rigidBodyOther,
		std::vector<Collision>& o_collisions
		)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
	}

	bool  PlaneCollider::intersectWho(const RigidBody& i_rigidBody,
		const PlaneCollider& i_colliderOther,
		const RigidBody& i_rigidBodyOther,
		std::vector<Collision>& o_collisions
		)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
	}

}
#pragma once

#include "Collider.h"
#include "Vector3.h"

#include <vector>

namespace PhysicEngine
{

	class SphereCollider;
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

		bool intersect	(	const RigidBody& i_rigidBody,
							const Collider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

		coordinate getAxis() const;

		float getAxisValue() const;

		lookDirections getLookingDirection() const;


	private:

		bool intersectWho	(	const RigidBody& i_rigidBody,
								const BoxCollider& i_colliderOther,
								const RigidBody& i_rigidBodyOther,
								std::vector<Collision>& o_collisions
							)	const;

		bool intersectWho	(	const RigidBody& i_rigidBody,
								const SphereCollider& i_colliderOther,
								const RigidBody& i_rigidBodyOther,
								std::vector<Collision>& o_collisions
							)	const;

		bool intersectWho	(	const RigidBody& i_rigidBody,
								const PlaneCollider& i_colliderOther,
								const RigidBody& i_rigidBodyOther,
								std::vector<Collision>& o_collisions
							)	const;

		coordinate axis;
		float axisValue;
		lookDirections lookDirection;

	};

}
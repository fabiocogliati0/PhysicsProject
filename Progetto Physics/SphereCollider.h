#pragma once

#include "Collider.h"

#include <vector>

namespace PhysicEngine{

	class BoxCollider;
	class RigidBody;
	struct Collision;

	class SphereCollider : public Collider
	{

	public:

		SphereCollider();

		SphereCollider(float radius);

		SphereCollider* clone() const;

		bool intersect	(	const RigidBody& i_rigidBody, 
							const Collider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

		float getRadius() const;

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

		float radius;

	};

}
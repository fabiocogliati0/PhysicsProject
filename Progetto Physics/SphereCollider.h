#pragma once

#include "Collider.h"

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
							Collision& o_collision
						)	const;

		float getRadius() const;

		const Utils::Vector3& getInertia() const;

		float getVolume() const;

	private:

		bool intersectWho	(	const RigidBody& i_rigidBody,
								const BoxCollider& i_colliderOther,
								const RigidBody& i_rigidBodyOther,
								Collision& o_collision
							)	const;

		bool intersectWho	(	const RigidBody& i_rigidBody,
								const SphereCollider& i_colliderOther,
								const RigidBody& i_rigidBodyOther,
								Collision& o_collision
							)	const;

		float radius;

	};

}
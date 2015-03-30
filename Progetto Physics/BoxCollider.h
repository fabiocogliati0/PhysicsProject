#pragma once

#include "Collider.h"
#include "Vector3.h"

namespace PhysicEngine
{

	class SphereCollider;
	struct Collision;
	class RigidBody;

	class BoxCollider : public Collider
	{

	public:

		BoxCollider();

		BoxCollider(float SemiX, float SemiY, float SemiZ);
		
		BoxCollider(const Utils::Vector3& size);

		BoxCollider* clone() const;

		bool intersect	(	const RigidBody& i_rigidBody, 
							const Collider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							Collision& o_collision
						)	const;

		const Utils::Vector3& getVertex(int vertex) const;

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


		Utils::Vector3 vertices[8];

	};

}
#pragma once

#include "Collider.h"
#include "Vector3.h"

#include <vector>

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
							std::vector<Collision>& o_collisions
						)	const;

		const Utils::Vector3& getVertex(int vertex) const;


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


		Utils::Vector3 vertices[8];

	};

}
#pragma once

#include "Vector3.h"

namespace PhysicEngine
{
	
	//Forward declarations
	class SphereCollider;
	class BoxCollider;
	struct Collision;
	class RigidBody;

	class Collider
	{

	public:


		virtual Collider* clone() const = 0;

		virtual bool intersect	(	const RigidBody& i_rigidBody, 
									const Collider& i_colliderOther,
									const RigidBody& i_rigidBodyOther,
									Collision& o_collision
								)	const = 0;

		virtual bool intersectWho	(	const RigidBody& i_rigidBody,
										const BoxCollider& i_colliderOther,
										const RigidBody& i_rigidBodyOther,
										Collision& o_collision
									)	const = 0;

		virtual bool intersectWho	(	const RigidBody& i_rigidBody,
										const SphereCollider& i_colliderOther,
										const RigidBody& i_rigidBodyOther,
										Collision& o_collision
									)	const = 0;

		virtual ~Collider();

		const Utils::Vector3& getRawInertia() const;

		float getVolume() const;

	protected:

		void setRawInertia(const Utils::Vector3& rawInertia);

		void setVolume(float volume);

	private:

		Utils::Vector3 rawInertia;
		
		float volume;

	};

}
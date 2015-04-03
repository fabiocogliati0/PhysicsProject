#pragma once

#include "Vector3.h"

#include <vector>

namespace PhysicEngine
{
	
	//Forward declarations
	class SphereCollider;
	class PlaneCollider;
	class BoxCollider;
	struct Collision;
	class RigidBody;

	class Collider
	{

	public:

		/*Public enum*/

		enum ColliderType
		{
			BoxColliderType = 0,
			SphereColliderType,
			PlaneColliderType
		};

		/*Pure Virtual Functions*/

		virtual ~Collider();

		virtual Collider* clone() const = 0;

		virtual ColliderType getColliderType() const = 0;



		/*Non-Virtual functions*/

		bool intersect	(	const RigidBody& i_rigidBody, 
							const Collider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

		bool intersect	(	const RigidBody& i_rigidBody,
							const BoxCollider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

		bool intersect	(	const RigidBody& i_rigidBody,
							const SphereCollider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

		bool intersect	(	const RigidBody& i_rigidBody,
							const PlaneCollider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

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
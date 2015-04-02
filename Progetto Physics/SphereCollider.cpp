#include "SphereCollider.h"
#include "IntersectOperations.h"
#include "Collider.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <cassert>

namespace PhysicEngine{

	class BoxCollider;

	SphereCollider::SphereCollider() : SphereCollider(1.0f)
	{
	}

	SphereCollider::SphereCollider(float radius) : radius(radius)
	{

		assert(radius > 0.0f);

		//calculates volume
		setVolume((4.0f * static_cast<float>(M_PI) *radius * radius * radius) / 3.0f);

		//calculates raw inertia
		Utils::Vector3 rawInertia;
		rawInertia.x = rawInertia.y = rawInertia.z = 0.4f * radius * radius;
		setRawInertia(rawInertia);
	}

	SphereCollider* SphereCollider::clone () const
	{
		return new SphereCollider(*this);
	}

	/*Itersects Operations*/

	bool SphereCollider::intersect	(	const RigidBody& i_rigidBody, 
										const Collider& i_colliderOther,
										const RigidBody& i_rigidBodyOther,
										std::vector<Collision>& o_collisions
									)	const
	{
		return i_colliderOther.intersectWho(i_rigidBody, *this, i_rigidBodyOther, o_collisions);
	}

	float SphereCollider::getRadius() const
	{
		return radius;
	}

	bool SphereCollider::intersectWho	(	const RigidBody& i_rigidBody,
											const BoxCollider& i_colliderOther,
											const RigidBody& i_rigidBodyOther,
											std::vector<Collision>& o_collisions
										)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
	}

	bool SphereCollider::intersectWho	(	const RigidBody& i_rigidBody,
											const SphereCollider& i_colliderOther,
											const RigidBody& i_rigidBodyOther,
											std::vector<Collision>& o_collisions
										)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
	}


	bool SphereCollider::intersectWho	(	const RigidBody& i_rigidBody,
											const PlaneCollider& i_colliderOther,
											const RigidBody& i_rigidBodyOther,
											std::vector<Collision>& o_collisions
										)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
	}
}
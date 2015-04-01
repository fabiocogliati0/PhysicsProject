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
		setVolume((4.0f * M_PI *radius * radius * radius) / 3.0f);

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
										Collision& o_collision
									)	const
	{
		return i_colliderOther.intersectWho(i_rigidBody, *this, i_rigidBodyOther, o_collision);
	}

	float SphereCollider::getRadius() const
	{
		return radius;
	}

	const Utils::Vector3& SphereCollider::getInertia() const
	{
		return Utils::Vector3::zero;	//todo : fabio
	}

	float SphereCollider::getVolume() const
	{
		return 1.0f;		//todo : fabio
	}

	bool SphereCollider::intersectWho	(	const RigidBody& i_rigidBody,
											const BoxCollider& i_colliderOther,
											const RigidBody& i_rigidBodyOther,
											Collision& o_collision
										)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collision);
	}

	bool SphereCollider::intersectWho	(	const RigidBody& i_rigidBody,
											const SphereCollider& i_colliderOther,
											const RigidBody& i_rigidBodyOther,
											Collision& o_collision
										)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collision);
	}

}
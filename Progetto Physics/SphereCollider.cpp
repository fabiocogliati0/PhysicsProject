#include "SphereCollider.h"

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

		//calculates area
		setArea(static_cast<float>(M_PI) * radius * radius);

		//calculates raw inertia
		Utils::Vector3 rawInertia;
		rawInertia.x = rawInertia.y = rawInertia.z = 0.4f * radius * radius;
		setRawInertia(rawInertia);
	}

	SphereCollider* SphereCollider::clone () const
	{
		return new SphereCollider(*this);
	}

	Collider::ColliderType SphereCollider::getColliderType() const
	{
		return SphereColliderType;
	}


	float SphereCollider::getRadius() const
	{
		return radius;
	}
}
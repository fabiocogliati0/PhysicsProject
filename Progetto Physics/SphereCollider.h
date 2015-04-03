#pragma once

#include "Collider.h"

#include <vector>

namespace PhysicEngine{

	class RigidBody;
	struct Collision;

	class SphereCollider : public Collider
	{

	public:

		SphereCollider();

		SphereCollider(float radius);

		SphereCollider* clone() const;

		ColliderType getColliderType() const;

		float getRadius() const;

	private:

		float radius;

	};

}
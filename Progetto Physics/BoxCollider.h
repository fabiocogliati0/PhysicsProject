#pragma once

#include "Collider.h"
#include "Vector3.h"

#include <vector>

namespace PhysicEngine
{
	//Forward declarations
	struct Collision;
	class RigidBody;

	class BoxCollider : public Collider
	{

	public:

		BoxCollider();

		BoxCollider(float SemiX, float SemiY, float SemiZ);
		
		BoxCollider(const Utils::Vector3& semi);

		BoxCollider* clone() const;

		ColliderType getColliderType() const;

		const Utils::Vector3& getSemiDimension() const;

		const Utils::Vector3& getVertex(int vertex) const;


	private:

		Utils::Vector3 vertices[8];

		Utils::Vector3 semiDimensions;

	};

}
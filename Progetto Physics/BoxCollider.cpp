#include "BoxCollider.h"

#include "Collider.h"
#include "Vector3.h"

namespace PhysicEngine
{

	BoxCollider::BoxCollider() : BoxCollider(0.5f, 0.5f, 0.5f)
	{
	}

	BoxCollider::BoxCollider(float semiX, float semiY, float semiZ) 
		: semiDimensions(Utils::Vector3(semiX,semiY, semiZ))
	{

		//font and back x coordinate
		vertices[0].x = vertices[4].x = -semiX;
		vertices[1].x = vertices[5].x = -semiX;
		vertices[2].x = vertices[6].x = semiX;
		vertices[3].x = vertices[7].x = semiX;

		//front and back y coordinate
		vertices[0].y = vertices[4].y = -semiY;
		vertices[1].y = vertices[5].y = semiY;
		vertices[2].y = vertices[6].y = semiY;
		vertices[3].y = vertices[7].y = -semiY;

		//front z coordinate
		vertices[0].z = semiZ;
		vertices[1].z = semiZ;
		vertices[2].z = semiZ;
		vertices[3].z = semiZ;

		//back z coordinate
		vertices[4].z = -semiZ;
		vertices[5].z = -semiZ;
		vertices[6].z = -semiZ;
		vertices[7].z = -semiZ;

		//set area
		setArea(2 * semiX * 2 * semiX);

		//set raw inertia
		Utils::Vector3 rawInertia;
		rawInertia.x = ((4.0f * semiY * semiY) + (4.0f * semiZ * semiZ)) / 12.0f;
		rawInertia.y = ((4.0f * semiX * semiX) + (4.0f * semiZ * semiZ)) / 12.0f;
		rawInertia.z = ((4.0f * semiX * semiX) + (4.0f * semiY * semiY)) / 12.0f;
		setRawInertia(rawInertia);

	}

	BoxCollider::BoxCollider(const Utils::Vector3& semi)
	{
		BoxCollider(semi.x, semi.y, semi.z);
	}

	BoxCollider* BoxCollider::clone() const
	{
		return new BoxCollider(*this);
	}

	Collider::ColliderType BoxCollider::getColliderType() const
	{
		return BoxColliderType;
	}

	const Utils::Vector3& BoxCollider::getSemiDimension() const
	{
		return semiDimensions;
	}

	const Utils::Vector3& BoxCollider::getVertex(int vertex) const
	{
		return vertices[vertex];
	}

}
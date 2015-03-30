#include "BoxCollider.h"
#include "IntersectOperations.h"
#include "Collider.h"
#include "Collision.h"

namespace PhysicEngine
{
	class SphereCollider;

	
	/*Constructors*/

	BoxCollider::BoxCollider() : BoxCollider(0.5f, 0.5f, 0.5f)
	{
	}

	BoxCollider::BoxCollider(float semiX, float semiY, float semiZ)
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

		//set volume
		setVolume(8.0f * semiX * semiY * semiZ);

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

	
	/*Itersects Operations*/

	bool BoxCollider::intersect	(	const RigidBody& i_rigidBody, 
									const Collider& i_colliderOther,
									const RigidBody& i_rigidBodyOther,
									Collision& o_collision
								)	const
	{
		return i_colliderOther.intersectWho(i_rigidBody, *this, i_rigidBodyOther, o_collision);
	}

	const Utils::Vector3& BoxCollider::getInertia() const
	{
		return Utils::Vector3::zero;	//todo : fabio
	}

	float BoxCollider::getVolume() const
	{
		return 1.0f;		//todo : fabio
	}

	const Utils::Vector3& BoxCollider::getVertex(int vertex) const
	{
		return vertices[vertex];
	}

	bool BoxCollider::intersectWho	(	const RigidBody& i_rigidBody,
										const BoxCollider& i_colliderOther,
										const RigidBody& i_rigidBodyOther,
										Collision& o_collision
									)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collision);
	}

	bool BoxCollider::intersectWho	(	const RigidBody& i_rigidBody,
										const SphereCollider& i_colliderOther,
										const RigidBody& i_rigidBodyOther,
										Collision& o_collision
									)	const
	{
		return IntersectOperations::intersect(*this, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collision);
	}

}
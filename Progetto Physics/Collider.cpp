#include "Collider.h"
#include "IntersectOperations.h"

#include "Vector3.h"

#include <cassert>

namespace PhysicEngine
{

	bool Collider::intersect(const RigidBody& i_rigidBody,
		const Collider& i_colliderOther,
		const RigidBody& i_rigidBodyOther,
		std::vector<Collision>& o_collisions
		)	const
	{
		if (this->getColliderType() == BoxColliderType){
			const BoxCollider& thisClass = reinterpret_cast<const BoxCollider&>(*this);
			return i_colliderOther.intersect(i_rigidBody, thisClass, i_rigidBodyOther, o_collisions);
		}
		else if (this->getColliderType() == SphereColliderType){
			const SphereCollider& thisClass = reinterpret_cast<const SphereCollider&>(*this);
			return i_colliderOther.intersect(i_rigidBody, thisClass, i_rigidBodyOther, o_collisions);
		}
		else if (this->getColliderType() == PlaneColliderType){
			const PlaneCollider& thisClass = reinterpret_cast<const PlaneCollider&>(*this);
			return i_colliderOther.intersect(i_rigidBody, thisClass, i_rigidBodyOther, o_collisions);
		}
		else{
			assert(false);
			return false;
		}
		
	}

	bool Collider::intersect(const RigidBody& i_rigidBody,
		const BoxCollider& i_colliderOther,
		const RigidBody& i_rigidBodyOther,
		std::vector<Collision>& o_collisions
		)	const
	{

		if (this->getColliderType() == BoxColliderType){
			const BoxCollider& thisClass = reinterpret_cast<const BoxCollider&>(*this);
			return IntersectOperations::intersect(thisClass, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
		}
		else if (this->getColliderType() == SphereColliderType){
			const SphereCollider& thisClass = reinterpret_cast<const SphereCollider&>(*this);
			return IntersectOperations::intersect(thisClass, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
		}
		else if (this->getColliderType() == PlaneColliderType){
			const PlaneCollider& thisClass = reinterpret_cast<const PlaneCollider&>(*this);
			return IntersectOperations::intersect(thisClass, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
		}
		else{
			assert(false);
			return false;
		}
	}

	bool Collider::intersect(const RigidBody& i_rigidBody,
		const SphereCollider& i_colliderOther,
		const RigidBody& i_rigidBodyOther,
		std::vector<Collision>& o_collisions
		)	const
	{
		if (this->getColliderType() == BoxColliderType){
			const BoxCollider& thisClass = reinterpret_cast<const BoxCollider&>(*this);
			return IntersectOperations::intersect(thisClass, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
		}
		else if (this->getColliderType() == SphereColliderType){
			const SphereCollider& thisClass = reinterpret_cast<const SphereCollider&>(*this);
			return IntersectOperations::intersect(thisClass, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
		}
		else if (this->getColliderType() == PlaneColliderType){
			const PlaneCollider& thisClass = reinterpret_cast<const PlaneCollider&>(*this);
			return IntersectOperations::intersect(thisClass, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
		}
		else{
			assert(false);
			return false;
		}
	}

	bool Collider::intersect(const RigidBody& i_rigidBody,
		const PlaneCollider& i_colliderOther,
		const RigidBody& i_rigidBodyOther,
		std::vector<Collision>& o_collisions
		)	const
	{
		if (this->getColliderType() == BoxColliderType){
			const BoxCollider& thisClass = reinterpret_cast<const BoxCollider&>(*this);
			return IntersectOperations::intersect(thisClass, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
		}
		else if (this->getColliderType() == SphereColliderType){
			const SphereCollider& thisClass = reinterpret_cast<const SphereCollider&>(*this);
			return IntersectOperations::intersect(thisClass, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
		}
		else if (this->getColliderType() == PlaneColliderType){
			const PlaneCollider& thisClass = reinterpret_cast<const PlaneCollider&>(*this);
			return IntersectOperations::intersect(thisClass, i_rigidBody, i_colliderOther, i_rigidBodyOther, o_collisions);
		}
		else{
			assert(false);
			return false;
		}
	}

	Collider::~Collider() {}

	const Utils::Vector3& Collider::getRawInertia() const
	{
		return rawInertia;
	}

	float Collider::getVolume() const
	{
		return volume;
	}

	void Collider::setRawInertia(const Utils::Vector3& rawInertia){
		this->rawInertia = rawInertia;
	}

	void Collider::setVolume(float volume)
	{
		this->volume = volume;
	}


}
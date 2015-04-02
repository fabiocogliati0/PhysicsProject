#pragma once

#include "PhyisicMaterial.h"
#include "Transform.h"

#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix.h"

#include <vector>

namespace PhysicEngine{

	//forward declarations
	class World;
	class Collider;
	struct Collision;

	class RigidBody{

	public:

		RigidBody	();

		RigidBody	(	float mass,
						const PhysicMaterial& material,
						const Collider& collider,
						bool isStatic = false
					);

		RigidBody	(	float mass,
						const PhysicMaterial& material,
						const Collider& collider,
						const Transform& transform,
						bool isStatic = false
					);

		RigidBody	(	float mass,
						const PhysicMaterial& material,
						const Collider& collider,
						const Transform& transform,
						const Utils::Vector3& velocity
					);

		RigidBody	(	float mass,
						const PhysicMaterial& material,
						const Collider& collider,
						const Transform& transform,
						const Utils::Vector3& velocity,
						const Utils::Vector3& angularVelocity
					);

		
		RigidBody(const RigidBody& other);
		
		void RigidBody::Init();

		~RigidBody();

		RigidBody& operator=(const RigidBody& other);

		bool intersect(const RigidBody& other, std::vector<Collision>& o_collisions) const;

		const Utils::Vector3& getPosition() const;

		const Utils::Matrix& getRotation() const;

		const Utils::Vector3& RigidBody::getVelocity() const;

		const Utils::Vector3& getAngularVelocity() const;

		Utils::Vector3 getInertia() const;

		float getVolume() const;

		bool isStatic() const;



		void addForce(const Utils::Vector3& point);

		void addForce(const Utils::Vector3& point, const Utils::Vector3& force);

		void updatePhyisic(float dt, const World& myWorld);

	private:

		PhysicMaterial material;
		
		Transform transform;
		
		Collider* collider;

		float mass;

		bool staticBody;

		Utils::Vector3 velocity;
		
		Utils::Vector3 angularVelocity;

		Utils::Matrix matrixRotation;			// Matrice di rotazione
		Utils::Quaternion quaternionRotation;	// Quaternione di rotazione

		// Temps
		Utils::Vector3 momentum;				// quantit� di moto
		Utils::Vector3 angularMomentum;			// momento angolare
		Utils::Vector3 resultantForce;			// forza risultante
		Utils::Vector3 resultantMomentum;		// momento risultante
		Utils::Vector3 gravity;					// Gravit�
		Utils::Vector3 velocityOfGravity;		// velocit� di gravit�
		bool updateForce;						// flag nel caso la forza venisse aggiornata

	};

}
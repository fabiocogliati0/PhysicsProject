/*
* Progetto di Physics Programming
* Fabio Cogliati, Manuele Nerucci
*/

#pragma once

#include "PhyisicMaterial.h"
#include "Transform.h"

#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix.h"

#include <vector>

namespace PhysicEngine{

	// forward declarations
	class World;
	class Collider;
	struct Collision;

	class RigidBody{

	public:

		enum staticBodyType
		{
			Static_Body = 1,
			Non_Static_Body
		};


		RigidBody	();

		RigidBody	(	float mass,
						const PhysicMaterial& material,
						const Collider& collider,
						staticBodyType isStatic = Non_Static_Body
					);

		RigidBody	(	float mass,
						const PhysicMaterial& material,
						const Collider& collider,
						const Transform& transform,
						staticBodyType isStatic = Non_Static_Body
					);

		RigidBody	(	float mass,
						const PhysicMaterial& material,
						const Collider& collider,
						const Transform& transform,
						const Utils::Vector3& velocity
					);

		
		RigidBody(const RigidBody& other);

		RigidBody& operator=(const RigidBody& other);

		~RigidBody();

		bool intersect(const RigidBody& other, std::vector<Collision>& o_collisions) const;


		const Utils::Vector3& getPosition() const;

		const Utils::Matrix& getRotation() const;

		const Utils::Vector3& RigidBody::getVelocity() const;

		const Utils::Vector3 getAngularVelocity() const;

		Utils::Vector3 getInertia() const;

		float getArea() const;

		float getElasticity() const;

		float getViscosity() const;

		float getFriction() const;

		staticBodyType getStaticBodyType() const;


		void setVelocity(const Utils::Vector3 &velocityInput);

		void addForceDT(const Utils::Vector3& point);

		void addForceDT(const Utils::Vector3& point, const Utils::Vector3& force);

		void updatePhyisic(float dt, const World& myWorld);

	private:

		PhysicMaterial material;
		
		Transform transform;
		
		Collider* collider;

		float mass;

		staticBodyType staticBody;

		Utils::Vector3 velocity;

		// Temps
		Utils::Vector3 momentum;				// quantit� di moto
		Utils::Vector3 angularMomentum;			// momento angolare
		Utils::Vector3 resultantForce;			// forza risultante
		Utils::Vector3 resultantMomentum;		// momento risultante
		Utils::Vector3 gravity;					// Gravit�
		Utils::Vector3 velocityOfGravity;		// velocit� di gravit�
		Utils::Vector3 angularVelocity;			// velocit� angolare
	};

}
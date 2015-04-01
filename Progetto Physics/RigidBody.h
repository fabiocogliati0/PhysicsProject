#pragma once

#include "PhyisicMaterial.h"
#include "Transform.h"
#include "Vector3.h"

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
						const Collider& collider
					);

		RigidBody	(	float mass,
						const PhysicMaterial& material,
						const Collider& collider,
						const Transform& transform
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
	
		~RigidBody();

		RigidBody& operator=(const RigidBody& other);

		bool intersect(const RigidBody& other, Collision& o_collision) const;


		const Utils::Vector3& getPosition() const;

		const Utils::Vector3& getRotation() const;	//todo: manuele : cosa deve tornare? la matrice o il vettore? o entrambe le versioni

		const Utils::Vector3& RigidBody::getVelocity() const;

		const Utils::Vector3& getAngularVelocity() const;

		Utils::Vector3 getInertia() const;

		float getVolume() const;



		void addForce(const Utils::Vector3& point);

		void addForce(const Utils::Vector3& point, const Utils::Vector3& force);

		void updatePhyisic(float dt, const World& myWorld);

	private:

		PhysicMaterial material;
		
		Transform transform;
		
		Collider* collider;

		float mass;

		Utils::Vector3 velocity;
		
		Utils::Vector3 angularVelocity;

		//temps-----------------------------------------------------------------------
		//todo: manuele - mettere i nomi in inglese e pensare quali servono e quali no
		Utils::Vector3 quantitaDiMoto;		//forza iniziale
		Utils::Vector3 momentoAngolare;		//momento angolare, ragionare su questo
		Utils::Vector3 forzaRisultante;
		Utils::Vector3 momentoRisultante;
		float matriceRotazione[9];			//todo: manuele - mettere la classe matrix?
		Utils::Vector3 gravity;				// Gravità
		Utils::Vector3 velocityOfGravity;	// velocità di gravità
		bool updateForce;

	};

}
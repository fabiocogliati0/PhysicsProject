/*
* Progetto di Physics Programming
* Fabio Cogliati, Manuele Nerucci
*/

#include "World.h"

#include "RigidBody.h"
#include "Collision.h"

#include <vector>
#include <cassert>

namespace PhysicEngine{

	// In caso di costruzione senza parametri viene inizializzato con la gravit�
	// di default e in assenza di aria (o di qualsiasi altro gas/fluido).
	World::World() 
		: World(0, Utils::Vector3(0.0f, -9.8f, 0.0f))
	{
	}

	World::World(float airDensity, const Utils::Vector3& gravityForce)
		: airDensity(airDensity), gravityForce(gravityForce)
	{
	}

	void World::updatePhysic(float dt)
	{
		std::vector<PhysicEngine::Collision> outputCollisions;

		for (size_t i = 0; i < (bodies.size() - 1); ++i)
		{
			for (size_t j = i + 1; j < bodies.size(); ++j)
			{
				bodies[i].intersect(bodies[j], outputCollisions);
				for (size_t k = 0; k < outputCollisions.size(); ++k)
				{
					PhysicEngine::Collision outputCollision = outputCollisions[k];

					// Seleziono la pi� grande fra le due
					float elasticity = bodies[i].getElasticity() >= bodies[j].getElasticity() ?
											bodies[i].getElasticity() : bodies[j].getElasticity();
					// Seleziono la pi� grande fra le due
					float viscosity = bodies[i].getViscosity() >= bodies[j].getViscosity() ? 
											bodies[i].getViscosity() : bodies[j].getViscosity();
					
					// Eseguo il prodotto fra i due attriti per trovare il totale se non sono uguali
					float friction = bodies[i].getFriction() == bodies[j].getFriction() ?
												bodies[i].getFriction()
												:
												bodies[i].getFriction() * bodies[j].getFriction();

					this->applyCollisionForce(bodies[i], bodies[j],
											  outputCollision, elasticity, viscosity, friction, dt);
				}
			}
		}
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			bodies[i].updatePhyisic(dt, *this);
		}
	}

	void World::applyCollisionForce(RigidBody &rigidBodyA, RigidBody &rigidBodyB,
									Collision collision, float elasticity, float viscosity,
									float friction, float dt) const
	{
		float modNormalVelocity;
		float modTangentVelocity;
		float force;
		Utils::Vector3 normalVelocity;
		Utils::Vector3 tangentVelocity;
		Utils::Vector3 normalForce;
		Utils::Vector3 tangentForce;
		Utils::Vector3 totalForce;

		Utils::Vector3 impactSpeed = collision.impactPoint - rigidBodyB.getPosition();
		impactSpeed = rigidBodyB.getAngularVelocity().cross(impactSpeed);
		impactSpeed = rigidBodyB.getVelocity() + impactSpeed;

		Utils::Vector3 temp = collision.impactPoint + rigidBodyA.getPosition();
		temp = rigidBodyA.getAngularVelocity().cross(temp);
		temp = rigidBodyA.getVelocity() + temp;

		impactSpeed = temp - impactSpeed;

		impactSpeed.invert();

		modNormalVelocity = impactSpeed.dot(collision.normal);
		normalVelocity = collision.normal * modNormalVelocity;
		tangentVelocity = impactSpeed - normalVelocity;

		force = (elasticity * collision.deformation) + (viscosity * modNormalVelocity);
		
		// Se � < 0 ho deformazione nulla
		force = force < 0 ? 0 : force;
		
		normalForce = collision.normal * force;
			
		force *= friction;
		
		tangentForce = tangentVelocity * force;

		modTangentVelocity = tangentVelocity.module();

		// Facendo il modulo della gravit� funziona anche per gravit� che non agisce su y
		// Scalo la forza tangente
		if (modTangentVelocity > gravityForce.module() * dt ) 
			tangentForce /= modTangentVelocity;
		else
			tangentForce /= gravityForce.module() * dt;
		totalForce = normalForce + tangentForce;
		
		Utils::Vector3 localPosition;

		localPosition = collision.impactPoint - rigidBodyA.getPosition();
		rigidBodyA.addForceDT(localPosition, totalForce);

		// Inverto la forza che sar� quella che ricever� l'oggetto B
		totalForce.invert();

		localPosition = collision.impactPoint - rigidBodyB.getPosition();
		rigidBodyB.addForceDT(localPosition, totalForce);
	}


	void World::addBody(const RigidBody& body)
	{
		bodies.push_back(body);
	}

	void World::removeBody(size_t index)
	{
		assert(index < bodies.size());

		bodies.erase(bodies.begin() + index);
	}

	const RigidBody& World::getBody(size_t index) const
	{
		assert(index < bodies.size());

		return bodies[index];
	}

	RigidBody& World::getBody(size_t index)
	{
		assert(index < bodies.size());

		return bodies[index];
	}

	size_t World::getNumberOfBodies() const
	{
		return bodies.size();
	}

	void World::setGravityForce(const Utils::Vector3 &gravityForce)
	{
		this->gravityForce = gravityForce;
	}

	const Utils::Vector3& World::getGravityForce() const
	{
		return this->gravityForce;
	}

	void World::setAirDensity(float airDensity)
	{
		this->airDensity = airDensity;
	}

	float World::getAirDensity() const
	{
		return airDensity;
	}
}
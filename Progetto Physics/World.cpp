#include "World.h"

#include "RigidBody.h"
#include "Collision.h"

#include <vector>
#include <cassert>

namespace PhysicEngine{

	World::World() 
		: World(1.0f, Utils::Vector3(0.0f, -9.8f, 0.0f))
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
					
					// Seleziono la più grande fra le due
					float elasticity = bodies[i].getElasticity() >= bodies[j].getElasticity() ?
											bodies[i].getElasticity() : bodies[j].getElasticity();

					float viscosity = bodies[i].getViscosity() >= bodies[j].getViscosity() ? 
											bodies[i].getViscosity() : bodies[j].getViscosity();
					
					// Eseguo il prodotto fra i due attriti per trovare il totale se non sono uguali
					float dynamicFricion = bodies[i].getDynamicFriction() == bodies[j].getDynamicFriction() ?
												bodies[i].getDynamicFriction()
												:
												bodies[i].getDynamicFriction() * bodies[j].getDynamicFriction();

					float staticFricion = bodies[i].getStaticFriction() == bodies[j].getStaticFriction() ?
												bodies[i].getStaticFriction()
												:
												bodies[i].getStaticFriction() * bodies[j].getStaticFriction();

					this->applyCollisionForce(bodies[i], bodies[j],
											  outputCollision, elasticity, viscosity, dynamicFricion, staticFricion, dt);
				}
			}
		}
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			if ( !bodies[i].isStatic() )
				bodies[i].updatePhyisic(dt, *this);
		}
	}

	void World::applyCollisionForce(RigidBody &rigidBodyA, RigidBody &rigidBodyB,
									Collision collision, float elasticity, float vicosity,
									float dynamicFricion, float staticFricion, float dt) const
	{
		float modNormalVelocity;
		float modTangentVelocity;
		float force;
		Utils::Vector3 normalVelocity;
		Utils::Vector3 tangentVelocity;
		Utils::Vector3 normalForce;
		Utils::Vector3 tangentForce;
		Utils::Vector3 totalForce;

		modNormalVelocity = collision.impactSpeed.dot(collision.normal);
		normalVelocity = collision.normal * modNormalVelocity;
		tangentVelocity = collision.impactSpeed - normalVelocity;
		force = (elasticity * collision.deformation) + (vicosity * modNormalVelocity);
		force = force < 0 ? 0 : force;
		normalForce = collision.normal * force;
		// Se entrami i corpi non si muovono uso l'attrito statico, altrimenti il dinamico
		if (rigidBodyA.getVelocity() == Utils::Vector3::zero && rigidBodyB.getVelocity() == Utils::Vector3::zero)
			force *= staticFricion;
		else
			force *= dynamicFricion;
		tangentForce = tangentVelocity * force;

		modTangentVelocity = tangentVelocity.module();

		if (modTangentVelocity > gravityForce.y * dt)
			tangentForce /= modTangentVelocity;
		else
			tangentForce /= gravityForce.y * dt;
		
		totalForce = normalForce + tangentForce;
		
		Utils::Vector3 localPosition;

		if ( !rigidBodyA.isStatic() )
		{
			localPosition = collision.impactPoint - rigidBodyA.getPosition();
			rigidBodyA.addForce(normalForce, localPosition);
		}

		normalForce.invert();

		if ( !rigidBodyB.isStatic() )
		{
			localPosition = collision.impactPoint - rigidBodyB.getPosition();
			rigidBodyB.addForce(normalForce, localPosition);
		}
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
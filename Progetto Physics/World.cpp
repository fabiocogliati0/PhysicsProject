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
					
					// Faccio il prodotto fra i due attriti per trovare il totale
					float dynamicFricion = bodies[i].getDynamicFriction() * bodies[j].getDynamicFriction();

					float staticFricion = bodies[i].getStaticFriction() * bodies[j].getStaticFriction();

					this->applyCollisionForce(bodies[i], bodies[j],
											 outputCollision, elasticity, viscosity, dynamicFricion, staticFricion);
				}
			}
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
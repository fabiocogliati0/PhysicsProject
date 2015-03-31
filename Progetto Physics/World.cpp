#include "World.h"

#include "RigidBody.h"

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
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			bodies[i].updatePhyisic(dt, *this);
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
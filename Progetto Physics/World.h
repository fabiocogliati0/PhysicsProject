#pragma once

#include "RigidBody.h"

#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix.h"

#include<vector>

namespace PhysicEngine{

	class World{

	public:

		World();

		World(float airDensity, const Utils::Vector3& gravityForce);

		
		void updatePhysic(float dt);


		void addBody(const RigidBody& body);

		void removeBody(size_t index);

		const RigidBody& getBody(size_t index) const;

		RigidBody& getBody(size_t index);

		size_t getNumberOfBodies() const;

		void setGravityForce(const Utils::Vector3& gravityForce);

		const Utils::Vector3& getGravityForce() const;

		void setAirDensity(float airDensity);

		float getAirDensity() const;

	private:

		void applyCollisionForce(RigidBody &rigidBodyA, RigidBody &RigidBodyB,
								 Collision collision, float elasticity, float vicosity,
								 float dynamicFricion, float staticFricion, float dt) const;

		std::vector<RigidBody> bodies;

		float airDensity;

		Utils::Vector3 gravityForce;

	};

}
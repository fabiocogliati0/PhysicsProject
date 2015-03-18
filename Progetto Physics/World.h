#pragma once

#include "RigidBody.h"

#include<vector>

class World{
	
private:

	float airDensity;				
	float gravityForce[3];

	std::vector<RigidBody> bodies;

public:

	World(float airDensity, const float gravityForce[3]);

	void updatePhysic(float dt);

	void addBody(const RigidBody& body);
	void removeBody(size_t index);
	RigidBody getBody(size_t index) const;
	size_t getNumberOfBody() const;

	void setGravityForce(const float gravityForce[3]);
	void getGravityForce(float o_gravityForce[3]) const;

	void setAirDensity(float airDensity);
	float getAirDensity() const;

};
#include "World.h"

World::World(float airDensity, const float gravityForce[3]){
	this->airDensity = airDensity;
	this->gravityForce[0] = gravityForce[0];
	this->gravityForce[1] = gravityForce[1];
	this->gravityForce[2] = gravityForce[2];
}

void World::setGravityForce(const float gravityForce[3]){
	this->gravityForce[0] = gravityForce[0];
	this->gravityForce[1] = gravityForce[1];
	this->gravityForce[2] = gravityForce[2];
}

void World::getGravityForce(float o_gravityForce[3]) const{
	o_gravityForce[0] = this->gravityForce[0];
	o_gravityForce[1] = this->gravityForce[1];
	o_gravityForce[2] = this->gravityForce[2];
}
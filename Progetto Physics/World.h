#pragma once

#include "PhysicsObject.h"

#include<vector>

class World{
	
private:

	float airDensity;			//attrito dell'aria (rho =~ 1). Rappresenta la densit� del fluido
	float gravityForce[3];		//forza di gravit�

	std::vector<PhysicsObject> objects;

public:

	void updatePhysic(float dt){
		for each objects
			objects.updatePhysic(dt);
	}

};
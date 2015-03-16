#pragma once

#include "PhysicsObject.h"

#include<vector>

class World{
	
private:

	float airDensity;			//attrito dell'aria (rho =~ 1). Rappresenta la densità del fluido
	float gravityForce[3];		//forza di gravità

	std::vector<PhysicsObject> objects;

public:

	void updatePhysic(float dt){
		for each objects
			objects.updatePhysic(dt);
	}

};
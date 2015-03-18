#pragma once

#include "PhyisicMaterial.h"
#include "Collider.h"

class RigidBody{

	//pensare se usare il pattern composite composite

private:

	PhysicMaterial* material;
	Collider* collider;

	//proprietà relative all'oggetto
	float mass;
	float inerzia[3];

	//stato
	float position[3];
	float rotation[3];
	float velocity[3];
	float angularVelocity[3];


	//temporanei
	float quantitaDiMoto[3];	//forza iniziale
	float momentoAngolare[3];	//momento angolare, ragionare su questo
	float forzaRisultante[3];
	float momentoRisultante[3];
	float matriceRotazione[9];

public:

	RigidBody(float mass,
		float inerzia[3],
		PhysicMaterial material,
		float position[3],
		float rotation[3],
		float velocity[3],
		float angularVelocity[3]);

	RigidBody(float mass, float inerzia[3], PhysicMaterial material);

	void addForce(float point[3], float force[3]);

	void updatePhyisic(float dt);

	void getPosition(float o_position[3]) const;
	void getRotation(float o_rotation[3]) const;
	void getVelocity(float o_velocity[3]) const;
	void getAngularVelocity(float o_angularVelocity[3]) const;
};
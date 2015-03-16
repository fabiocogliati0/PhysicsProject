#pragma once

class PhysicsObject{

	//pensare se usare il pattern composite composite

private:

	//proprietà oggetto
	float mass;
	float inerzia[3];
	float componenteElastica;
	float componenteViscosa;
	float coefficienteAttritoDinamico;
	float coefficienteAttritoStatico;

	//stato
	float position[3];
	float rotation[3];
	float velocity[3];
	float angularVelocity[3];

	//forze
	float impulse[3];			//forza iniziale
	float momentoAngolare[3];	//momento angolare, ragionare su questo

	// temporanei
	float forzaRisultante[3];
	float momentoRisultante[3];
	float matriceRotazione[9];

public:

	PhysicsObject(	float mass,
					float inerzia[3],
					float componenteElastica,
					float componenteViscosa,
					float coefficienteAttritoDinamico,
					float coefficienteAttritoStatico,
					float position[3],
					float rotation[3],
					float velocity[3],
					float angularVelocity[3])
	{

					impulse = 0;
					momentoAngolare = 0;
	}
	
	PhysicsObject(	float mass,
					float inerzia[3],
					float componenteElastica,
					float componenteViscosa,
					float coefficienteAttritoDinamico,
					float coefficienteAttritoStatico)
	{

		PhysicsObject();	//chiama quello sopra
	}

	void addForce(float point[3], float force[3]){

	}

	void updatePhyisic(float dt){
		//tutto il calcolo della fisica
	}


	//funzioni getter per il motore grafico

	float[3] getPosition();
	float[4x4] getRotation();


	//funzione getter per la logica di gioco

	float[3] getVelocity();
	float[3] getAngularVelocity();
};
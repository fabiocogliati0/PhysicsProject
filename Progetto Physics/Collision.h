#pragma once

#include "Vector3.h"

namespace PhysicEngine
{
	/*Struct che rappresenta una collisione, le funzioni di intersezione restituiscono questo oggeto*/
	struct Collision
	{
		float deformation;				//Deformazione della collisione
		Utils::Vector3 impactPoint;		//Punto di impatto della collisione
		Utils::Vector3 normal;			/*Normale al punto di impatto per calcolare la forza da applicare a un oggetto
										  e da invertire per calcolare la forza da applicare all'altro*/
	};

}
/*
* Progetto di Physics Programming
* Fabio Cogliati, Manuele Nerucci
*/

#pragma once

#include "Collider.h"

namespace PhysicEngine
{

	//Forward declarations
	struct Collision;

	/*Classe che rappresenta un plane collider con funzione Ax + By + Cz = 0*/
	class PlaneCollider : public Collider
	{

	public:

		/*Enum che rappresenta il verso del piano, MajorLookDirection : il piano guarda per funzione piano > 0
		  MinorLookDirection : il piano guarda per funzione pinao < 0*/
		enum lookDirections
		{
			MajorLookDirection = 0,
			MinorLookDirection
		};

		/*Costruttore di base*/
		PlaneCollider();

		/*Costruttore che vuole i parametri della funzione di piano e la direzione di sguardo*/
		PlaneCollider(float A, float B, float C, float D, lookDirections lookDirection);

		/*Funzione che clona l'oggetto tramite una new */
		PlaneCollider* clone() const;

		/*Ritorna il tipo di collider secondo l'enum ColliderType contenuto nella classe Collider */
		ColliderType getColliderType() const;

		/*Ritorna il coefficiente A della funzione di piano*/
		float getAFunctionCoefficient() const;

		/*Ritorna il coefficiente B della funzione di piano*/
		float getBFunctionCoefficient() const;

		/*Ritorna il coefficiente C della funzione di piano*/
		float getCFunctionCoefficient() const;

		/*Ritorna il coefficiente D della funzione di piano*/
		float getDFunctionCoefficient() const;

		/*Ritorna il verso di sguardo*/
		lookDirections getLookingDirection() const;


	private:

		/*coefficiente A della funzione di piano*/
		float A;

		/*coefficiente B della funzione di piano*/
		float B;

		/*coefficiente C della funzione di piano*/
		float C;

		/*coefficiente D della funzione di piano*/
		float D;

		/*direzione di sguardo della funzione di piano*/
		lookDirections lookDirection;

	};

}
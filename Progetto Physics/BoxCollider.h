/*
* Progetto di Physics Programming
* Fabio Cogliati, Manuele Nerucci
*/

#pragma once

#include "Collider.h"
#include "Vector3.h"

namespace PhysicEngine
{
	//Forward declarations
	struct Collision;

	/*Classe che rappresenta un box collider*/
	class BoxCollider : public Collider
	{

	public:

		/*Costruttore di base */
		BoxCollider();

		/*Costuttore che vuole le semidimensioni su ogni asse */
		BoxCollider(float SemiX, float SemiY, float SemiZ);
		
		/*Costuttore che vuole le semidimensioni su ogni asse contenute in un Vector3 */
		BoxCollider(const Utils::Vector3& semi);


		/*Funzione che clona l'oggetto tramite una new */
		BoxCollider* clone() const;

		/*Ritorna il tipo di collider secondo l'enum ColliderType contenuto nella classe Collider */
		ColliderType getColliderType() const;

		/*Ritorna un Vector3 contenente le tre semidimensioni della box */
		const Utils::Vector3& getSemiDimension() const;

		/*Ritorna la posizione locale del vertice di indice "vertex" */
		const Utils::Vector3& getVertex(int vertex) const;


	private:

		/*Vertici della box in coordinate locali */
		Utils::Vector3 vertices[8];

		/*Semidimensioni della box */
		Utils::Vector3 semiDimensions;

	};

}
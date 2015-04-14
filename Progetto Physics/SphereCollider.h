#pragma once

#include "Collider.h"

namespace PhysicEngine{

	//Forward declarations
	struct Collision;

	/*Classe che rappresenta uno sphere collider*/
	class SphereCollider : public Collider
	{

	public:

		/*Costruttore di base*/
		SphereCollider();

		/*Costruttore che vuole il raggio della sfera*/
		SphereCollider(float radius);

		/*Funzione che clona l'oggetto tramite una new */
		SphereCollider* clone() const;

		/*Ritorna il tipo di collider secondo l'enum ColliderType contenuto nella classe Collider */
		ColliderType getColliderType() const;

		/*Funzione che ritorna il raggio della sphera*/
		float getRadius() const;

	private:

		/*il raggio della sfera*/
		float radius;

	};

}
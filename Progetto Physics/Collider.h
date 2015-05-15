/*
* Progetto di Physics Programming
* Fabio Cogliati, Manuele Nerucci
*/

#pragma once

#include "Vector3.h"
#include <vector>

namespace PhysicEngine
{
	
	//Forward declarations
	class SphereCollider;
	class PlaneCollider;
	class BoxCollider;
	struct Collision;
	class RigidBody;

	/*Classe che rappresenta un collider generico, ogni collider (box,sphere,..) deriverà da questa classe */
	class Collider
	{

	public:

		/*Enum che rappresenta il tipo di collider */
		enum ColliderType
		{
			BoxColliderType = 0,
			SphereColliderType,
			PlaneColliderType
		};

		/*Distruttore */
		virtual ~Collider();

		/*Funzione che clona l'oggetto tramite una new */
		virtual Collider* clone() const = 0;

		/*Ritorna il tipo di collider secondo l'enum ColliderType contenuto nella classe Collider */
		virtual ColliderType getColliderType() const = 0;


		/*Funzione di intersezione che verifica la intersezione tra questo collider e un collider generico*/
		bool intersect	(	const RigidBody& i_rigidBody, 
							const Collider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

		/*Funzione di intersezione che verifica la intersezione tra questo collider e un box collider*/
		bool intersect	(	const RigidBody& i_rigidBody,
							const BoxCollider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

		/*Funzione di intersezione che verifica la intersezione tra questo collider e uno sphere collider*/
		bool intersect	(	const RigidBody& i_rigidBody,
							const SphereCollider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

		/*Funzione di intersezione che verifica la intersezione tra questo collider e un plane collider*/
		bool intersect	(	const RigidBody& i_rigidBody,
							const PlaneCollider& i_colliderOther,
							const RigidBody& i_rigidBodyOther,
							std::vector<Collision>& o_collisions
						)	const;

		/*Restitusce il punto di inerzia tenendo conto della geometria dell'oggetto senza tenere conto della massa*/
		const Utils::Vector3& getRawInertia() const;

		/*Restituisce l'area del collider*/
		float getArea() const;


	protected:

		/*Funzione che setta il punto di inerzia dell'oggetto, verrà chiamata dalle classi derivate*/
		void setRawInertia(const Utils::Vector3& rawInertia);

		/*Funzione che setta l'area dell'oggetto, verrà chiamata dalle classi derivate*/
		void setArea(float area);


	private:

		/*Punto di inerzia dell'oggetto senza tenere conto della massa*/
		Utils::Vector3 rawInertia;
		
		/*area dell'oggetto*/
		float area;

	};

}
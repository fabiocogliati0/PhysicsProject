#pragma once

#include "SphereCollider.h"
#include "BoxCollider.h"
#include "PlaneCollider.h"
#include "RigidBody.h"
#include "Collision.h"
#include <vector>
#include <cmath>

namespace PhysicEngine
{
	/*Classe statica che gestisce le collisioni, contiene la funzione intersect, templata e specializzata per ogni
	  coppia di tipi di collider */
	class IntersectOperations
	{

	public:

		/*Funzione di intersezione per collider di tipo generico, poichè per i tipi T e U passati non è stata trovata
		  una specializzazione di intersect che accetta quei tipi questa funzione richiama intersect per girare i parametri
		  es: se viene chiamato intersect su Sphere/Box non viene trovata nessuna specializzazione quindi viene chiamata
		  questa funzione che gira i parametri e invocando Box/Sphere che è una specializzazione di intersect esistente
		  o_collisions è un vector che conterrà tutte le collisioni trovate */
		template <class T, class U> static bool  intersect	(	const T& i_collider1,
																const RigidBody& i_rigidBody1,
																const U& i_collider2,
																const RigidBody& i_rigidBody2,
																std::vector<Collision>& o_collisions
															)
		{
			bool isIntersection = intersect(i_collider2, i_rigidBody2, i_collider1, i_rigidBody1, o_collisions);
			if (isIntersection)
			{
				for (size_t i = 0; i < o_collisions.size(); ++i)
				{
					//la normale viene invertita perchè è sempre riferita rispetto al primo oggetto passato
					o_collisions[i].normal.invert();
				}
			}
			return isIntersection;
		}

		/*Funzione di intersezione specializzata per il caso Sphere/Sphere */
		template<> static bool intersect<SphereCollider, SphereCollider>	(	const SphereCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const SphereCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{

			bool intersect;

			o_collisions.clear();

			Collision o_collision;

			const Utils::Vector3& spherePosition1 = i_rigidBody1.getPosition();
			float sphereRadius1 = i_collider1.getRadius();

			const Utils::Vector3& spherePosition2 = i_rigidBody2.getPosition();
			float sphereRadius2 = i_collider2.getRadius();

			float distanceFromCenters = (spherePosition1 - spherePosition2).module();;
			float radiusSum = sphereRadius1 + sphereRadius2;

			//se la distanza dai centri è maggiore del raggio allora non vi è intersezione
			if (distanceFromCenters > radiusSum)
			{
				intersect = false;
			}
			else if (distanceFromCenters > 0.0001f)
			{
				o_collision.deformation = radiusSum - distanceFromCenters;
				o_collision.impactPoint = (spherePosition1 + spherePosition2) / 2.0f;
				o_collision.normal = o_collision.impactPoint - spherePosition2;
				o_collision.normal.normalize();

				o_collisions.push_back(o_collision);

				intersect = true;
			}
			else
			{
				/*se la distnaza dai centri è prossima allo 0 è impossibile calcolare la normale, conviene quindi scartare la collisione
				  essa verrà rilevata a un successivo dt*/
				intersect = false;
			}

			return intersect;
		}

		/*Funzione di intersezione specializzata per il caso Plane/Plane. La funzione non rileva mai una collisione poichè i piani
		  sono utilizzati come oggetti statici, per questo ritorna sempre false */
		template<> static bool intersect<PlaneCollider, PlaneCollider>		(	const PlaneCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const PlaneCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{
			o_collisions.clear();
			return false;
		}

		/*Funzione di intersezione specializzata per il caso Box/Sphere */
		template<> static bool intersect<BoxCollider, SphereCollider>		(	const BoxCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const SphereCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{

			bool intersection;
			
			o_collisions.clear();

			Collision o_collision;

			const Utils::Vector3& boxPosition = i_rigidBody1.getPosition();
			const Utils::Vector3& spherePosition = i_rigidBody2.getPosition();

			float sphereRadius = i_collider2.getRadius();

			//calcolo il probabile punto di impatto
			o_collision.impactPoint = spherePosition - boxPosition;
			o_collision.impactPoint = i_rigidBody1.getRotation().RotateRelative(o_collision.impactPoint);
			if (o_collision.impactPoint.x > i_collider1.getVertex(2).x) o_collision.impactPoint.x = i_collider1.getVertex(2).x;
			if (o_collision.impactPoint.x < i_collider1.getVertex(0).x) o_collision.impactPoint.x = i_collider1.getVertex(0).x;
			if (o_collision.impactPoint.y > i_collider1.getVertex(1).y) o_collision.impactPoint.y = i_collider1.getVertex(1).y;
			if (o_collision.impactPoint.y < i_collider1.getVertex(0).y) o_collision.impactPoint.y = i_collider1.getVertex(0).y;
			if (o_collision.impactPoint.z > i_collider1.getVertex(0).z) o_collision.impactPoint.y = i_collider1.getVertex(0).z;
			if (o_collision.impactPoint.z < i_collider1.getVertex(4).z) o_collision.impactPoint.z = i_collider1.getVertex(4).z;
			o_collision.impactPoint = i_rigidBody1.getRotation().RotateAbsolute(o_collision.impactPoint);
			o_collision.impactPoint = o_collision.impactPoint + boxPosition;
			
			//calcolo la probabile normale
			o_collision.normal = o_collision.impactPoint - spherePosition;
			
			//calcolo la distanza tra il punto di impatto e posizione della sphera
			float d = o_collision.normal.module();

			//se la distanza tra impatto e sfera è più grande del raggio della sfera allora non vi è intersezione
			if (d > sphereRadius)
			{
				intersection = false;
			}
			else if (d > 0.0001f)
			{
				o_collision.deformation = sphereRadius - d;
				o_collision.normal = o_collision.normal / d;

				o_collisions.push_back(o_collision);

				intersection = true;
			}
			else
			{
				//se la distanza tra punto di intersezione e centro della sfera è prossima allo 0 meglio scartare la collisione
				intersection = false;
			}

			return intersection;
			
		}

		/*Funzione di intersezione specializzata per il caso Box/Plane */
		template<> static bool intersect<BoxCollider, PlaneCollider>		(	const BoxCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const PlaneCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{

			bool intersect = false;

			o_collisions.clear();
			
			const Utils::Vector3& boxPosition = i_rigidBody1.getPosition();
			const Utils::Matrix& boxRotation = i_rigidBody1.getRotation();
			const Utils::Vector3& boxVelocity = i_rigidBody1.getVelocity();
			const Utils::Vector3& boxAngVelocity = i_rigidBody1.getAngularVelocity();

			float A = i_collider2.getAFunctionCoefficient();
			float B = i_collider2.getBFunctionCoefficient();
			float C = i_collider2.getCFunctionCoefficient();
			float D = i_collider2.getDFunctionCoefficient();

			PlaneCollider::lookDirections look = i_collider2.getLookingDirection();

			//genero una collisione per ogni vertice che entra in intersezione con il piano
			for (unsigned short i = 0; i < 8; ++i)
			{
				//calcolo il probabile punto di impatto
				Collision o_collision;
				o_collision.impactPoint = boxRotation.RotateAbsolute(i_collider1.getVertex(i));
				o_collision.impactPoint = boxPosition + o_collision.impactPoint;

				//calcolo la deformation
				o_collision.deformation = -(A * o_collision.impactPoint.x 
											+ B * o_collision.impactPoint.y
											+ C * o_collision.impactPoint.z + D);

				//se il piano guarda verso l'alto e la deformazione è positiva allora ho una collisione
				//se il piano guarda verso i basso e la deformazione è negativa allora ho una collisione
				if ((o_collision.deformation > 0 && look == PlaneCollider::MajorLookDirection)
					|| (o_collision.deformation < 0 && look == PlaneCollider::MinorLookDirection)
					)
				{
					o_collision.normal.x = A;
					o_collision.normal.y = B;
					o_collision.normal.z = C;

					if (look == PlaneCollider::MinorLookDirection)
					{
						//se il piano guarda verso il basso giro normale e deformazione
						o_collision.normal.invert();
						o_collision.deformation = -o_collision.deformation;
					}

					o_collisions.push_back(o_collision);

					intersect = true;
				}

			}

			return intersect;
		}

		/*Funzione di intersezione specializzata per il caso Sphere/Plane */
		template<> static bool intersect<SphereCollider, PlaneCollider>		(	const SphereCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const PlaneCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{
			bool intersection;

			o_collisions.clear();
			
			float sphereRadius = i_collider1.getRadius();
			const Utils::Vector3& spherePosition = i_rigidBody1.getPosition();

			float A = i_collider2.getAFunctionCoefficient();
			float B = i_collider2.getBFunctionCoefficient();
			float C = i_collider2.getCFunctionCoefficient();
			float D = i_collider2.getDFunctionCoefficient();

			PlaneCollider::lookDirections look = i_collider2.getLookingDirection();

			Collision o_collision;

			//calcolo il probabile impact point
			if (look == PlaneCollider::MajorLookDirection)
			{
				o_collision.impactPoint.x = -A;
				o_collision.impactPoint.y = -B;
				o_collision.impactPoint.z = -C;
			}
			else
			{
				o_collision.impactPoint.x = A;
				o_collision.impactPoint.y = B;
				o_collision.impactPoint.z = C;
			}
			o_collision.impactPoint *= sphereRadius;
			o_collision.impactPoint = spherePosition + o_collision.impactPoint;

			//calcolo la probabile deformation
			o_collision.deformation = -(A * o_collision.impactPoint.x 
											+ B * o_collision.impactPoint.y
											+ C * o_collision.impactPoint.z + D);

			//se il piano guarda verso l'alto e la deformazione è positiva allora ho una collisione
			//se il piano guarda verso i basso e la deformazione è negativa allora ho una collisione
			if (( o_collision.deformation > 0 && look == PlaneCollider::MajorLookDirection)
				|| (o_collision.deformation < 0 && look == PlaneCollider::MinorLookDirection)
				)
			{
				o_collision.normal.x = A;
				o_collision.normal.y = B;
				o_collision.normal.z = C;

				if (look == PlaneCollider::MinorLookDirection)
				{
					//se il piano guarda verso il basso giro normale e deformazione
					o_collision.normal.invert();
					o_collision.deformation = -o_collision.deformation;
				}

				o_collisions.push_back(o_collision);

				intersection = true;
			}
			else
			{
				intersection = false;
			}
			
			return intersection;
		}


		/*Funzione di intersezione specializzata per il caso Box/Box */
		template<> static bool intersect<BoxCollider, BoxCollider>			(	const BoxCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const BoxCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{
			o_collisions.clear();
			bool intersection;
			
			//calcoliamo le collisioni del secondo box sul primo
			intersection = checkBoxBoxIntersect(i_collider1, i_rigidBody1, i_collider2, i_rigidBody2, o_collisions);

			//calcoliamo le collisioni del primo box sul secondo
			intersection = intersection || checkBoxBoxIntersect(i_collider2, i_rigidBody2, i_collider1, i_rigidBody1, o_collisions);

			return intersection;
		}

	private:

		/*Funzione di intersezione che trova le intersezioni del secondo box passato sul primo */
		static bool checkBoxBoxIntersect	(	const BoxCollider& i_collider1,
												const RigidBody& i_rigidBody1,
												const BoxCollider& i_collider2,
												const RigidBody& i_rigidBody2,
												std::vector<Collision>& o_collisions
											)
		{
			bool intersection = false;

			const Utils::Vector3& boxPosition1 = i_rigidBody1.getPosition();
			const Utils::Matrix& boxRotation1 = i_rigidBody1.getRotation();
			const Utils::Vector3& boxSemiDim1 = i_collider1.getSemiDimension();

			const Utils::Vector3& boxPosition2 = i_rigidBody2.getPosition();
			const Utils::Matrix& boxRotation2 = i_rigidBody2.getRotation();
			const Utils::Vector3& boxSemiDim2 = i_collider2.getSemiDimension();

			Utils::Vector3 radiusSum = boxSemiDim1 + boxSemiDim2;

			//vettore che collega il secondo box al primo
			Utils::Vector3 centersDistance = boxPosition1 - boxPosition2;

			//vettore che rappresenta il centro del secondo box nel primo
			Utils::Vector3 secondCenteredInFirst = i_rigidBody1.getRotation().RotateRelative(boxPosition2);

			//verifico se le sfere circoscritte nei due box non si toccano, in questo modo posso rigettare l'intersezione senza fare ulteriori calcoli
			bool sphereConsideration = (centersDistance.x * centersDistance.x < radiusSum.x * radiusSum.x)
				&& (centersDistance.y * centersDistance.y < radiusSum.y * radiusSum.y)
				&& (centersDistance.z * centersDistance.z < radiusSum.z * radiusSum.z)
				&& (centersDistance.module() < radiusSum.module());

			if (sphereConsideration)
			{
				//porto ogni vertice del box2 nello spazio locale del box1
				
				Utils::Vector3 vertex[8];

				for (unsigned short i = 0; i < 8; ++i)
				{
					//prendo la posizione del vertice locale nel primo oggetto
					vertex[i] = i_collider2.getVertex(i);							

					//trasformo la posizione in globale
					vertex[i] = boxRotation2.RotateAbsolute(vertex[i]);
					vertex[i] += boxPosition2;

					//metto il vertice nello spazio locale dell'altro oggetto
					vertex[i] -= boxPosition1;
					vertex[i] = boxRotation1.RotateRelative(vertex[i]);
					
				}

				//calcolo il minimo e il massimo locale per il primo box
				Utils::Vector3 min = boxSemiDim1;
				min.invert();
				Utils::Vector3 max = boxSemiDim1;

				//per ogni vertice del secondo verifico se sta dentro nello spazio del primo
				for (unsigned short i = 0; i < 8; ++i)
				{
					bool isInside = (vertex[i].x >= min.x && vertex[i].y >= min.y && vertex[i].z >= min.z)
						&& (vertex[i].x <= max.x && vertex[i].y <= max.y && vertex[i].z <= max.z);

					if (isInside)
					{
					
						Collision o_collision;

						//il punto di impatto sta sul vertice, lo riporto in posizione globale
						o_collision.impactPoint = vertex[i];
						o_collision.impactPoint = boxRotation1.RotateAbsolute(o_collision.impactPoint);
						o_collision.impactPoint += boxPosition1;

						//la normale è la componente più grande del vettore che collega i due centri
						o_collision.normal = centersDistance;
						o_collision.normal.normalize();

						if (o_collision.normal.x * o_collision.normal.x > o_collision.normal.y * o_collision.normal.y)
						{
							if (o_collision.normal.x * o_collision.normal.x > o_collision.normal.z * o_collision.normal.z)
								o_collision.normal = Utils::Vector3(o_collision.normal.x, 0.0f, 0.0f);
							else
								o_collision.normal = Utils::Vector3(0.0f, 0.0f, o_collision.normal.z);
						}
						else
						{
							if (o_collision.normal.y * o_collision.normal.y > o_collision.normal.z * o_collision.normal.z)
								o_collision.normal = Utils::Vector3(0.0f, o_collision.normal.y, 0.0f);
							else
								o_collision.normal = Utils::Vector3(0.0f, 0.0f, o_collision.normal.z);
						}

						o_collision.normal.normalize();

						//la deformazione è la semidimensione del cubo meno la proiezione della distanza dal centro al punto di impatto
						Utils::Vector3 distanceBox1fromCenter = o_collision.impactPoint - boxPosition1;
						if (o_collision.normal.x * o_collision.normal.x > 0.0f)
							o_collision.deformation = boxSemiDim1.x - abs((o_collision.normal.dot(distanceBox1fromCenter)));
						else if (o_collision.normal.y * o_collision.normal.y > 0.0f)
							o_collision.deformation = boxSemiDim1.y - abs((o_collision.normal.dot(distanceBox1fromCenter)));
						else if (o_collision.normal.z * o_collision.normal.z > 0.0f)
							o_collision.deformation = boxSemiDim1.z - abs((o_collision.normal.dot(distanceBox1fromCenter)));

						if (o_collision.deformation < 0.0001f || o_collision.normal.module() <0.0001f)
							continue;

						o_collisions.push_back(o_collision);

						intersection = true;
					
					}
				}

			}

			return intersection;
		}

		/*Costruttore messo privato per non permettere di costruire un'istanza della classe */
		IntersectOperations();

	};

}
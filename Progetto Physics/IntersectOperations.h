#pragma once

#include "SphereCollider.h"
#include "BoxCollider.h"
#include "RigidBody.h"
#include "Collision.h"

#include <iostream>	//todo: rimuovere

namespace PhysicEngine
{

	class IntersectOperations
	{

	public:

		template <class T, class U> static bool  intersect	(	const T& i_collider1,
																const RigidBody& i_rigidBody1,
																const U& i_collider2,
																const RigidBody& i_rigidBody2,
																Collision& o_collision
															)
		{
			return intersect(i_collider2, i_rigidBody2, i_collider1, i_rigidBody1, o_collision);
		}

		template<> static bool intersect<SphereCollider, SphereCollider>	(	const SphereCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const SphereCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				Collision& o_collision
																			)
		{
			Utils::Vector3 spherePosition1 = i_rigidBody1.getPosition();
			float sphereRadius1 = i_collider1.getRadius();

			Utils::Vector3 spherePosition2 = i_rigidBody2.getPosition();
			float sphereRadius2 = i_collider2.getRadius();

			float distanceFromCenters = (spherePosition1 - spherePosition2).module();;
			float radiusSum = sphereRadius1 + sphereRadius2;

			if (distanceFromCenters > radiusSum)
			{
				return false;
			}
			else
			{
				o_collision.deformation = radiusSum - distanceFromCenters;
				o_collision.impactPoint = (spherePosition1 + spherePosition2) / 2.0f;

				o_collision.normal = o_collision.impactPoint - spherePosition2;
				o_collision.normal.normalize();
				o_collision.normal = o_collision.normal / distanceFromCenters;	//todo: perchè? è giusta sta roba?

				o_collision.impactSpeed = o_collision.impactPoint - spherePosition2;
				o_collision.impactSpeed = i_rigidBody2.getAngularVelocity().cross(o_collision.impactSpeed);
				o_collision.impactSpeed = i_rigidBody2.getVelocity() + o_collision.impactSpeed;

				Utils::Vector3 temp = o_collision.impactPoint + spherePosition1;
				temp = i_rigidBody1.getAngularVelocity().cross(temp);
				temp = i_rigidBody1.getVelocity() + temp;

				o_collision.impactSpeed = temp - o_collision.impactSpeed;

				return true;
			}
		}

		template<> static bool intersect<BoxCollider, BoxCollider>			(	const BoxCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const BoxCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				Collision& o_collision
																			)
		{
			//todo
			return false;
		}

		template<> static bool intersect<BoxCollider, SphereCollider>		(	const BoxCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const SphereCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				Collision& o_collision
																			)
		{

			Utils::Vector3 boxPosition = i_rigidBody1.getPosition();
			Utils::Vector3 spherePosition = i_rigidBody2.getPosition();
			float sphereRadius = i_collider2.getRadius();

			o_collision.impactPoint = spherePosition - boxPosition;
			//RuotaRelative(r->MRot, c[0].PuntoImpatto, c[0].PuntoImpatto);	//todo

			if (o_collision.impactPoint.x > i_collider1.getVertex(2).x) o_collision.impactPoint.x = i_collider1.getVertex(2).x;
			if (o_collision.impactPoint.x < i_collider1.getVertex(0).x) o_collision.impactPoint.x = i_collider1.getVertex(0).x;
			if (o_collision.impactPoint.y > i_collider1.getVertex(1).y) o_collision.impactPoint.y = i_collider1.getVertex(1).y;
			if (o_collision.impactPoint.y < i_collider1.getVertex(0).y) o_collision.impactPoint.y = i_collider1.getVertex(0).y;
			if (o_collision.impactPoint.z > i_collider1.getVertex(0).z) o_collision.impactPoint.y = i_collider1.getVertex(0).z;
			if (o_collision.impactPoint.z < i_collider1.getVertex(4).z) o_collision.impactPoint.z = i_collider1.getVertex(4).z;

			//RuotaAssolute(r->MRot, c[0].PuntoImpatto, c[0].PuntoImpatto); //todo
			o_collision.impactPoint = o_collision.impactPoint + boxPosition;
			
			o_collision.normal = o_collision.impactPoint - spherePosition;
			float d = o_collision.normal.module();

			if (d > sphereRadius)
			{
				return false;
			}
			else
			{
				o_collision.deformation = sphereRadius - d;
				o_collision.normal = o_collision.normal / d;

				o_collision.impactSpeed = o_collision.impactPoint - spherePosition;
				o_collision.impactSpeed = i_rigidBody2.getAngularVelocity().cross(o_collision.impactSpeed);
				o_collision.impactSpeed = i_rigidBody2.getVelocity() + o_collision.impactSpeed;

				Utils::Vector3 temp = o_collision.impactPoint + boxPosition;
				temp = i_rigidBody1.getAngularVelocity().cross(temp);
				temp = i_rigidBody1.getVelocity() + temp;

				o_collision.impactSpeed = temp - o_collision.impactSpeed;

				return true;
			}
			
		}

	private:

		IntersectOperations();

	};

}
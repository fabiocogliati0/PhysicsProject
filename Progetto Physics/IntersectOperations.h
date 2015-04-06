#pragma once

#include "SphereCollider.h"
#include "BoxCollider.h"
#include "PlaneCollider.h"
#include "RigidBody.h"
#include "Collision.h"

#include <vector>

namespace PhysicEngine
{

	class IntersectOperations
	{

	public:

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
				for (size_t i = 0; i < o_collisions.size(); ++i) o_collisions[i].normal *= -1.0f;	
			}
			return isIntersection;
		}

		template<> static bool intersect<SphereCollider, SphereCollider>	(	const SphereCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const SphereCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{

			o_collisions.clear();

			Collision o_collision;

			const Utils::Vector3& spherePosition1 = i_rigidBody1.getPosition();
			float sphereRadius1 = i_collider1.getRadius();

			const Utils::Vector3& spherePosition2 = i_rigidBody2.getPosition();
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
				o_collision.normal = o_collision.normal / distanceFromCenters;

				o_collision.impactSpeed = o_collision.impactPoint - spherePosition2;
				o_collision.impactSpeed = i_rigidBody2.getAngularVelocity().cross(o_collision.impactSpeed);
				o_collision.impactSpeed = i_rigidBody2.getVelocity() + o_collision.impactSpeed;

				Utils::Vector3 temp = o_collision.impactPoint + spherePosition1;
				temp = i_rigidBody1.getAngularVelocity().cross(temp);
				temp = i_rigidBody1.getVelocity() + temp;

				o_collision.impactSpeed = temp - o_collision.impactSpeed;

				o_collisions.push_back(o_collision);

				return true;
			}
		}

		template<> static bool intersect<BoxCollider, BoxCollider>			(	const BoxCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const BoxCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{
			o_collisions.clear();

			int numberOfCollisions = 0;

			for (int i = 0; i < 8; ++i)
			{
				Collision o_collision;

				o_collision.impactPoint = i_rigidBody2.getRotation().RotateAbsolute(i_collider2.getVertex(i));

				//COME CAVOLO SI FA???
			}


			return false;
		}

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

		template<> static bool intersect<BoxCollider, SphereCollider>		(	const BoxCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const SphereCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{
			
			o_collisions.clear();

			Collision o_collision;

			const Utils::Vector3& boxPosition = i_rigidBody1.getPosition();
			const Utils::Vector3& spherePosition = i_rigidBody2.getPosition();
			float sphereRadius = i_collider2.getRadius();

			o_collision.impactPoint = spherePosition - boxPosition;
			//RuotaRelative(r->MRot, c[0].PuntoImpatto, c[0].PuntoImpatto);	//
			o_collision.impactPoint = i_rigidBody1.getRotation().RotateRelative(o_collision.impactPoint);


			if (o_collision.impactPoint.x > i_collider1.getVertex(2).x) o_collision.impactPoint.x = i_collider1.getVertex(2).x;
			if (o_collision.impactPoint.x < i_collider1.getVertex(0).x) o_collision.impactPoint.x = i_collider1.getVertex(0).x;
			if (o_collision.impactPoint.y > i_collider1.getVertex(1).y) o_collision.impactPoint.y = i_collider1.getVertex(1).y;
			if (o_collision.impactPoint.y < i_collider1.getVertex(0).y) o_collision.impactPoint.y = i_collider1.getVertex(0).y;
			if (o_collision.impactPoint.z > i_collider1.getVertex(0).z) o_collision.impactPoint.y = i_collider1.getVertex(0).z;
			if (o_collision.impactPoint.z < i_collider1.getVertex(4).z) o_collision.impactPoint.z = i_collider1.getVertex(4).z;

			//RuotaAssolute(r->MRot, c[0].PuntoImpatto, c[0].PuntoImpatto); //
			o_collision.impactPoint = i_rigidBody1.getRotation().RotateAbsolute(o_collision.impactPoint);
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

				o_collisions.push_back(o_collision);

				return true;
			}
			
		}

		template<> static bool intersect<BoxCollider, PlaneCollider>		(	const BoxCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const PlaneCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{
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

			bool isCollision = false;

			for (int i = 0; i < 8; ++i)
			{
				Collision o_collision;
				o_collision.impactPoint = boxRotation.RotateAbsolute(i_collider1.getVertex(i));
				o_collision.impactSpeed = boxAngVelocity.cross(o_collision.impactPoint);
				o_collision.impactPoint = boxPosition + o_collision.impactPoint;
				o_collision.impactSpeed = boxVelocity + o_collision.impactSpeed;

				o_collision.deformation = -(A * o_collision.impactPoint.x 
											+ B * o_collision.impactPoint.y
											+ C * o_collision.impactPoint.z + D);

				if ((o_collision.deformation >= 0 && look == PlaneCollider::MajorLookDirection)
					|| (o_collision.deformation <= 0 && look == PlaneCollider::MinorLookDirection)
					)
				{
					o_collision.normal.x = A;
					o_collision.normal.y = B;
					o_collision.normal.z = C;

					if (look == PlaneCollider::MinorLookDirection)
					{
						o_collision.normal *= -1.0f;
						o_collision.deformation *= -1.0f;
					}

					o_collision.impactSpeed = o_collision.impactSpeed * -1.0f;

					o_collisions.push_back(o_collision);

					isCollision = true;
				}

			}

			return isCollision;
		}

		template<> static bool intersect<SphereCollider, PlaneCollider>		(	const SphereCollider& i_collider1,
																				const RigidBody& i_rigidBody1,
																				const PlaneCollider& i_collider2,
																				const RigidBody& i_rigidBody2,
																				std::vector<Collision>& o_collisions
																			)
		{
			o_collisions.clear();
			
			float sphereRadius = i_collider1.getRadius();
			const Utils::Vector3& spherePosition = i_rigidBody1.getPosition();
			const Utils::Vector3& sphereVelocity = i_rigidBody1.getVelocity();
			const Utils::Vector3& sphereAngVelocity = i_rigidBody1.getAngularVelocity();

			float A = i_collider2.getAFunctionCoefficient();
			float B = i_collider2.getBFunctionCoefficient();
			float C = i_collider2.getCFunctionCoefficient();
			float D = i_collider2.getDFunctionCoefficient();

			PlaneCollider::lookDirections look = i_collider2.getLookingDirection();

			Collision o_collision;

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

			o_collision.impactSpeed = sphereAngVelocity.cross(o_collision.impactPoint);
			o_collision.impactPoint = spherePosition + o_collision.impactPoint;
			o_collision.impactSpeed = sphereVelocity + o_collision.impactSpeed;

			o_collision.deformation = -(A * o_collision.impactPoint.x 
											+ B * o_collision.impactPoint.y
											+ C * o_collision.impactPoint.z + D);

			/*if (!((look == PlaneCollider::MajorLookDirection && o_collision.deformation > 0)
				|| (look == PlaneCollider::MinorLookDirection && o_collision.deformation < 0)))*/
			if (( o_collision.deformation >= 0 && look == PlaneCollider::MajorLookDirection)
				|| (o_collision.deformation <= 0 && look == PlaneCollider::MinorLookDirection)
				)
			{
				o_collision.normal.x = A;
				o_collision.normal.y = B;
				o_collision.normal.z = C;

				if (look == PlaneCollider::MinorLookDirection)
				{
					o_collision.normal *= -1.0f;
					o_collision.deformation *= -1.0f;
				}

				o_collision.impactSpeed *= -1.0f;

				o_collisions.push_back(o_collision);

				return true;
			}
			else
			{
				return false;
			}
			
		}

	private:

		IntersectOperations();

	};

}
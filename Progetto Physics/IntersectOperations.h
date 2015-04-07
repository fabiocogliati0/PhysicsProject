#pragma once

#include "SphereCollider.h"
#include "BoxCollider.h"
#include "PlaneCollider.h"
#include "RigidBody.h"
#include "Collision.h"

#include <vector>
#include <algorithm>

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
				for (size_t i = 0; i < o_collisions.size(); ++i)
				{
					o_collisions[i].normal.invert();
					o_collisions[i].impactSpeed.invert();
				}
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

			bool intersection = false;

			const Utils::Vector3& boxPosition1 = i_rigidBody1.getPosition();
			const Utils::Matrix& boxRotation1 = i_rigidBody1.getRotation();
			const Utils::Vector3& boxSemiDim1 = i_collider1.getSemiDimension();

			const Utils::Vector3& boxPosition2 = i_rigidBody2.getPosition();
			const Utils::Matrix& boxRotation2 = i_rigidBody2.getRotation();
			const Utils::Vector3& boxSemiDim2 = i_collider2.getSemiDimension();

			Utils::Vector3 centersDistance = boxPosition2 - boxPosition1;
			centersDistance *= -1.0f;

			//verificare se porta in object space, chiedere a manu
			Utils::Vector3 secondCenteredInFirst = i_rigidBody1.getRotation().RotateRelative(boxPosition2);			
			Utils::Vector3 radiusSum = boxSemiDim1 + boxSemiDim2;
			
			//SphereTest for early reject
			bool sphereConsideration = (centersDistance.x * centersDistance.x  < radiusSum.x * radiusSum.x)
									&& (centersDistance.y * centersDistance.y  < radiusSum.y * radiusSum.y)
									&& (centersDistance.z * centersDistance.z  < radiusSum.z * radiusSum.z)
									&& (centersDistance.module() < radiusSum.module());

			if (!sphereConsideration)
			{
				//compute min and max for first box
				Utils::Vector3 min = boxPosition1 - boxSemiDim1;
				Utils::Vector3 max = boxPosition1 + boxSemiDim1;

				min = boxRotation1.RotateRelative(min);	//todo: stessa roba
				max = boxRotation1.RotateRelative(max);

				Utils::Vector3 SecondHalfSizeInFistSystem(boxSemiDim2);

				SecondHalfSizeInFistSystem = boxRotation2.RotateRelative(SecondHalfSizeInFistSystem);
				SecondHalfSizeInFistSystem = boxRotation1.RotateRelative(SecondHalfSizeInFistSystem);

				//second vs first
				Utils::Vector3 vertex[8];
				for (int i = 0; i < 8; ++i)
				{
					vertex[i] = secondCenteredInFirst;

					bool signX, signY, signZ;

					switch (i)
					{
					case 0:
						signX = 1;
						signY = 1;
						signZ = 1;
						break;
					case 1:
						signX = 1;
						signY = 1;
						signZ = -1;
						break;
					case 2:
						signX = 1;
						signY = -1;
						signZ = 1;
						break;
					case 3:
						signX = 1;
						signY = -1;
						signZ = -1;
						break;
					case 4:
						signX = -1;
						signY = 1;
						signZ = 1;
						break;
					case 5:
						signX = -1;
						signY = 1;
						signZ = -1;
						break;
					case 6:
						signX = -1;
						signY = -1;
						signZ = 1;
						break;
					case 7:
						signX = -1;
						signY = -1;
						signZ = -1;
						break;
					default:
						break;
					}

					vertex[i].x += SecondHalfSizeInFistSystem.x * signX;
					vertex[i].y += SecondHalfSizeInFistSystem.y * signY;
					vertex[i].z += SecondHalfSizeInFistSystem.z * signZ;
				}

				//Debug
				const Utils::Vector3 debug1 = SecondHalfSizeInFistSystem * i_collider1.getVertex(0) + secondCenteredInFirst;
				const Utils::Vector3 debug2 = SecondHalfSizeInFistSystem * i_collider2.getVertex(0) + secondCenteredInFirst;
				//---Debug


				//compute points inside
				int indexes[8];
				float compenetration[8];
				unsigned int  pointsInside = 0;
				bool isInside = false;

				for (unsigned int i = 0; i < 8; ++i)
				{
					isInside =	(vertex[i].x > min.x && vertex[i].y > min.y && vertex[i].z > min.z)
							&&	(vertex[i].x < max.x && vertex[i].y < max.y && vertex[i].z < max.z);

					if (isInside)
					{
						indexes[pointsInside] = i;
						compenetration[pointsInside] = (vertex[i] - boxPosition1).module();
						++pointsInside;	
					}
				}


				//Compute Collision Data iff there is a collision
				if (pointsInside)
				{
					float CompenetrationSum = compenetration[0];
					vertex[indexes[0]] *= compenetration[0];

					//centroid of points which is the point of collision impact
					for (unsigned int i = 1; i < pointsInside; ++i)
					{
						vertex[indexes[0]] += vertex[indexes[i]];
						CompenetrationSum += compenetration[i];
						compenetration[0] = std::fmaxf(compenetration[0], compenetration[i]);
					}

					vertex[indexes[0]] /= CompenetrationSum;

					centersDistance.normalize();

					if (centersDistance.x > centersDistance.y)
					{
						if (centersDistance.x > centersDistance.z)
						{
							centersDistance = Utils::Vector3(centersDistance.x, 0.0f, 0.0f);
						}
						else
						{
							centersDistance = Utils::Vector3(0.0f, 0.0f, centersDistance.z);
						}
					}
					else
					{
						if (centersDistance.y > centersDistance.z)
						{
							centersDistance = Utils::Vector3(0.0f, centersDistance.y, 0.0f);
						}
						else
						{
							centersDistance = Utils::Vector3(0.0f, 0.0f, centersDistance.z);
						}
					}

					centersDistance.normalize();

					Collision o_collision;
					o_collision.deformation = compenetration[0];
					o_collision.impactPoint = vertex[indexes[0]];
					o_collision.normal = centersDistance;
					o_collision.impactSpeed = o_collision.normal;		//todo;

					o_collisions.push_back(o_collision);

					intersection = true;

				}
				else
				{
					//First vs Second
					Utils::Vector3 vertex[8];

					for (int i = 0; i < 8; ++i)
					{
						vertex[i] = secondCenteredInFirst;

						bool signX, signY, signZ;

						switch (i)
						{
						case 0:
							signX = 1;
							signY = 1;
							signZ = 1;
							break;
						case 1:
							signX = 1;
							signY = 1;
							signZ = -1;
							break;
						case 2:
							signX = 1;
							signY = -1;
							signZ = 1;
							break;
						case 3:
							signX = 1;
							signY = -1;
							signZ = -1;
							break;
						case 4:
							signX = -1;
							signY = 1;
							signZ = 1;
							break;
						case 5:
							signX = -1;
							signY = 1;
							signZ = -1;
							break;
						case 6:
							signX = -1;
							signY = -1;
							signZ = 1;
							break;
						case 7:
							signX = -1;
							signY = -1;
							signZ = -1;
							break;
						default:
							break;
						}

						vertex[i].x += SecondHalfSizeInFistSystem.x * signX;
						vertex[i].y += SecondHalfSizeInFistSystem.y * signY;
						vertex[i].z += SecondHalfSizeInFistSystem.z * signZ;
					}

					//compute points inside
					int indexes[8];
					float compenetration[8];
					unsigned int  pointsInside = 0;
					bool isInside = false;

					for (unsigned int i = 0; i < 8; ++i)
					{
						isInside = (vertex[i].x > min.x && vertex[i].y > min.y && vertex[i].z > min.z)
							&& (vertex[i].x < max.x && vertex[i].y < max.y && vertex[i].z < max.z);

						if (isInside)
						{
							indexes[pointsInside] = i;
							compenetration[pointsInside] = (vertex[i] - boxPosition2).module();
							++pointsInside;
						}
					}


					if (pointsInside)
					{
						float CompenetrationSum = compenetration[0];
						vertex[indexes[0]] *= compenetration[0];

						//centroid of points which is the point of collision impact
						for (unsigned int i = 1; i < pointsInside; ++i)
						{
							vertex[indexes[0]] += vertex[indexes[i]];
							CompenetrationSum += compenetration[i];
							compenetration[0] = std::fmaxf(compenetration[0], compenetration[i]);
						}

						vertex[indexes[0]] /= CompenetrationSum;

						centersDistance.normalize();

						if (centersDistance.x > centersDistance.y)
						{
							if (centersDistance.x > centersDistance.z)
							{
								centersDistance = Utils::Vector3(centersDistance.x, 0.0f, 0.0f);
							}
							else
							{
								centersDistance = Utils::Vector3(0.0f, 0.0f, centersDistance.z);
							}
						}
						else
						{
							if (centersDistance.y > centersDistance.z)
							{
								centersDistance = Utils::Vector3(0.0f, centersDistance.y, 0.0f);
							}
							else
							{
								centersDistance = Utils::Vector3(0.0f, 0.0f, centersDistance.z);
							}
						}

						centersDistance.normalize();

						Collision o_collision;
						o_collision.deformation = compenetration[0];
						o_collision.impactPoint = vertex[indexes[0]];
						o_collision.normal = centersDistance;
						o_collision.impactSpeed = o_collision.normal;		//todo;

						o_collisions.push_back(o_collision);

						intersection = true;
					}
					else
					{
						intersection = false;
					}
				}

			}


			return intersection;
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
			o_collision.impactPoint = i_rigidBody1.getRotation().RotateRelative(o_collision.impactPoint);

			if (o_collision.impactPoint.x > i_collider1.getVertex(2).x) o_collision.impactPoint.x = i_collider1.getVertex(2).x;
			if (o_collision.impactPoint.x < i_collider1.getVertex(0).x) o_collision.impactPoint.x = i_collider1.getVertex(0).x;
			if (o_collision.impactPoint.y > i_collider1.getVertex(1).y) o_collision.impactPoint.y = i_collider1.getVertex(1).y;
			if (o_collision.impactPoint.y < i_collider1.getVertex(0).y) o_collision.impactPoint.y = i_collider1.getVertex(0).y;
			if (o_collision.impactPoint.z > i_collider1.getVertex(0).z) o_collision.impactPoint.y = i_collider1.getVertex(0).z;
			if (o_collision.impactPoint.z < i_collider1.getVertex(4).z) o_collision.impactPoint.z = i_collider1.getVertex(4).z;

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
						o_collision.normal.invert();
						o_collision.deformation = -o_collision.deformation;
					}

					o_collision.impactSpeed.invert();

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
					o_collision.normal.invert();
					o_collision.deformation = -o_collision.deformation;
				}

				o_collision.impactSpeed.invert();

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
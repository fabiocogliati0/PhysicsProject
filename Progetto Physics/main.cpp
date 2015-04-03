
#include <iostream>
#include <vector>

#include "World.h"
#include "PhyisicMaterial.h"
#include "BoxCollider.h"
#include "SphereCollider.h"
#include "PlaneCollider.h"
#include "Collision.h"

using namespace PhysicEngine;
using namespace Utils;

int main()
{

	World world(10.0f, Vector3::zero);

	PhysicMaterial material;
	material.dynamicFriction = 1.0f;
	material.elasticity = 1.0f;
	material.staticFriction = 1.0f;
	material.viscosity = 1.0f;

	float mass = 10.0f;

	Vector3 inertia;

	PhysicEngine::BoxCollider a(1.0f,1.0f,1.0f);
	PhysicEngine::SphereCollider b(3.0f);
	PhysicEngine::PlaneCollider c(0.0f, 1.0f, 0.0f, 0.0f, PlaneCollider::MajorLookDirection);	//y>0

	RigidBody rigidbody1(1.0f, material, a);
	RigidBody rigidbody2(1.0f, material, b);
	RigidBody rigidBody3(1.0f, material, c);

	std::vector<Collision> outputCollision;
	rigidbody1.intersect(rigidbody2, outputCollision);

	//world.addBody(rigidbody1);
	//world.addBody(rigidbody2);

	std::getchar();
}
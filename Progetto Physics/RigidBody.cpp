#include "RigidBody.h"

#include "World.h"
#include "Collider.h"
#include "BoxCollider.h"

#include <cassert>
#include <vector>

namespace PhysicEngine
{

	RigidBody::RigidBody()
		: RigidBody(1.0f, PhysicMaterial(), BoxCollider(), Transform(), Utils::Vector3::zero, Utils::Vector3::zero)
	{
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider,
		bool isStatic
		)
		: RigidBody(mass, material, collider, Transform(), Utils::Vector3::zero, Utils::Vector3::zero)
	{
		staticBody = isStatic;
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider,
		const Transform& transform,
		bool isStatic
		)
		: RigidBody(mass, material, collider, transform, Utils::Vector3::zero, Utils::Vector3::zero)
	{
		staticBody = isStatic;
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider,
		const Transform& transform,
		const Utils::Vector3& velocity
		)
		: RigidBody(mass, material, collider, transform, velocity, Utils::Vector3::zero)
	{	
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider,
		const Transform& transform,
		const Utils::Vector3& velocity,
		const Utils::Vector3& angularVelocity
		) 
		:	mass(mass), 
			material(material),
			transform(transform), 
			velocity(velocity), 
			angularVelocity(angularVelocity),
			staticBody(false)
	{
		this->collider = collider.clone();
		
		this->Init();
	}


	RigidBody::RigidBody(const RigidBody& other)
		:	mass(other.mass),
			material(other.material),
			transform(other.transform),
			staticBody(other.staticBody),
			velocity(other.velocity),
			angularVelocity(other.angularVelocity)
	{

		assert(other.collider != nullptr);
		
		this->collider = other.collider->clone();
	}

	void RigidBody::Init()
	{
		quaternionRotation.s = 1;
		transform.rotationMatrix[0] = transform.rotationMatrix[4] = transform.rotationMatrix[8] = 1;
	}

	RigidBody::~RigidBody()
	{
		delete collider;
		collider = nullptr;
	}

	RigidBody& RigidBody::operator=(const RigidBody& other)
	{
		if (this != &other)
		{
			this->mass = other.mass;
			this->material = other.material;
			this->transform = other.transform;
			this->staticBody = other.staticBody;
			this->velocity = other.velocity;
			this->angularVelocity = other.angularVelocity;

			assert(other.collider != nullptr);
			this->collider = other.collider->clone();
		}

		return *this;
	}

	bool RigidBody::intersect(const RigidBody& other, std::vector<Collision>& o_collisions) const
	{
		assert(collider != nullptr);
		return collider->intersect(*this, *(other.collider), other, o_collisions);
	}

	const Utils::Vector3& RigidBody::getPosition() const
	{
		return transform.position;
	}

	const Utils::Matrix& RigidBody::getRotation() const
	{
		return transform.rotationMatrix;
	}

	const Utils::Vector3& RigidBody::getVelocity() const
	{
		return velocity;
	}

	const Utils::Vector3& RigidBody::getAngularVelocity() const
	{
		return angularVelocity;
	}

	Utils::Vector3 RigidBody::getInertia() const
	{
		assert(collider != nullptr);
		return collider->getRawInertia() * mass;
	}

	float RigidBody::getVolume() const
	{
		assert(collider != nullptr);
		return collider->getVolume();
	}
	
	float RigidBody::getElasticity() const
	{
		return this->material.elasticity;
	}

	float RigidBody::getViscosity() const
	{
		return this->material.viscosity;
	}

	float RigidBody::getDynamicFriction() const
	{
		return this->material.dynamicFriction;
	}

	float RigidBody::getStaticFriction() const
	{
		return this->material.staticFriction;
	}

	bool RigidBody::isStatic() const
	{
		return staticBody;
	}

	void RigidBody::addForce(const Utils::Vector3& force)
	{
		this->addForce(Utils::Vector3::zero, force);
	}

	void RigidBody::addForce(const Utils::Vector3& point, const Utils::Vector3& force)
	{
		// Sommo la forza ricevuta a quella che ho già
		this->resultantForce += force;
		if (point != Utils::Vector3::zero)
		{
			// *** Calcolo il momento risultante che mi servirà per ruotare l'oggetto
			Utils::Vector3 newResultantMomentum = point.cross(force);
			resultantMomentum += newResultantMomentum;
		
		}
	}

	void RigidBody::updatePhyisic(float dt, const World& myWorld)
	{
		Utils::Quaternion newQuaternionRotation;
		Utils::Vector3 tmpAngularVelocity;

		// Se la gravità è uguale l'ho già calcolata e mi evito una moltiplicazione
		if (myWorld.getGravityForce() != gravity)
		{
			// *** calcolo nuovamente la quantità di moto della gravità
			// quantitadiMotoGravity = gravity * dt;
			gravity = myWorld.getGravityForce();
			velocityOfGravity = gravity * dt;
		}

		// Moto rettilineo uniforme
		if (resultantForce != Utils::Vector3::zero)
		{
			momentum = resultantForce * dt; 
			velocity += momentum / mass;
		}

		velocity += velocityOfGravity;
		transform.position += velocity * dt;

		// Moto angolare
		if (resultantMomentum != Utils::Vector3::zero)
		{
			angularMomentum = resultantMomentum * dt;

			// Per risolvere problemi di inerzia, "raddrizzo" il mio
			// oggetto, altrimenti l'inerzia cambierebbe in base a come è disposto l'oggetto
			tmpAngularVelocity = transform.rotationMatrix.RotateRelative(angularMomentum);

			// Ho effettivamente l'angularVelocity ora 
			float debug = this->getInertia().z;

			tmpAngularVelocity.x /= this->getInertia().x;
			tmpAngularVelocity.y /= this->getInertia().y;
			tmpAngularVelocity.z /= this->getInertia().z;
		}

		// Sommo l'angularVelocity settata con quella derivata dalle forze di collisione (tmpAngularVelocity)
		angularVelocity += tmpAngularVelocity;

		if (angularVelocity != Utils::Vector3::zero)
		{
			newQuaternionRotation.set(1, angularVelocity.x * dt / 2, angularVelocity.y * dt / 2,
									  angularVelocity.z * dt / 2);

			// Normalizzo il quaternione per utilizzarlo per la rotazione
			newQuaternionRotation.normalize();

			quaternionRotation *= newQuaternionRotation;
			quaternionRotation.normalize();

			// La velocità la ritorno in assoluto
			angularVelocity = transform.rotationMatrix.RotateAbsolute(angularVelocity);

			// Creo la matrice attuale di rotazione da quaternione ricavato tramite velocità angolare
			quaternionRotation.toMatrix(transform.rotationMatrix);
		}

		// Azzeramento forza risultante e momento risultante 
		resultantForce = resultantMomentum = Utils::Vector3::zero;
	}
}
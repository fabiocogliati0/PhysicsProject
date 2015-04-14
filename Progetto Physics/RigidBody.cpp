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
		staticBodyType isStatic
		)
		: RigidBody(mass, material, collider, Transform(), Utils::Vector3::zero, Utils::Vector3::zero)
	{
		staticBody = isStatic;
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider,
		const Transform& transform,
		staticBodyType isStatic
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
			staticBody(Non_Static_Body)
	{
		// TEMP
		this->momentum = Utils::Vector3::zero;
		this->angularMomentum = Utils::Vector3::zero;
		this->resultantForce = Utils::Vector3::zero;
		this->resultantMomentum = Utils::Vector3::zero;
		this->gravity = Utils::Vector3::zero;
		this->velocityOfGravity = Utils::Vector3::zero;
		
		this->collider = collider.clone();
	}


	RigidBody::RigidBody(const RigidBody& other)
		:	mass(other.mass),
			material(other.material),
			transform(other.transform),
			staticBody(other.staticBody),
			velocity(other.velocity),
			angularVelocity(other.angularVelocity)
	{
		// TEMP
		this->momentum = other.momentum;
		this->angularMomentum = other.angularMomentum;
		this->gravity = other.gravity;
		this->velocityOfGravity = other.velocityOfGravity;

		assert(other.collider != nullptr);
		this->collider = other.collider->clone();
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

			// TEMP
			this->momentum = other.momentum;
			this->angularMomentum = other.angularMomentum;
			this->gravity = other.gravity;
			this->velocityOfGravity = other.velocityOfGravity;

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
		return transform.getPosition();
	}

	const Utils::Matrix& RigidBody::getRotation() const
	{
		return transform.getRotationMatrix();
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

	RigidBody::staticBodyType RigidBody::getStaticBodyType() const
	{
		return staticBody;
	}

	void RigidBody::setVelocity(const Utils::Vector3 &input)
	{
		velocity = input;
	}

	void RigidBody::setAngularVelocity(const Utils::Vector3 &input)
	{
		angularMomentum = input * this->getInertia();
	}

	void RigidBody::addForceDT(const Utils::Vector3& force)
	{
		if (!this->staticBody)
		{
			this->addForceDT(Utils::Vector3::zero, force);
		}
	}

	void RigidBody::addForceDT(const Utils::Vector3& point, const Utils::Vector3& force)
	{
		if (!this->staticBody)
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
	}

	void RigidBody::updatePhyisic(float dt, const World& myWorld)
	{
		Utils::Quaternion newRotationQuaternion;
		Utils::Quaternion totalRotationQuaternion = transform.getRotationQuaternion();
		Utils::Matrix rotationMatrix = transform.getRotationMatrix();

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
		transform.setPosition(transform.getPosition() + (velocity * dt));

		// Moto angolare
		if (resultantMomentum != Utils::Vector3::zero)
		{
			angularMomentum += resultantMomentum * dt;
			// Per risolvere problemi di inerzia, "raddrizzo" il mio
			// oggetto, altrimenti l'inerzia cambierebbe in base a come è disposto l'oggetto
			angularVelocity = rotationMatrix.RotateRelative(angularMomentum);

			// Ho effettivamente l'angularVelocity ora 
			angularVelocity.x /= this->getInertia().x;
			angularVelocity.y /= this->getInertia().y;
			angularVelocity.z /= this->getInertia().z;
		}

		if (angularVelocity != Utils::Vector3::zero)
		{
			newRotationQuaternion.set(1, angularVelocity.x * dt / 2, angularVelocity.y * dt / 2,
									  angularVelocity.z * dt / 2);

			// Normalizzo il quaternione per utilizzarlo per la rotazione
			newRotationQuaternion.normalize();

			totalRotationQuaternion *= newRotationQuaternion;
			totalRotationQuaternion.normalize();

			// La velocità la ritorno in assoluto
			angularVelocity = rotationMatrix.RotateAbsolute(angularVelocity);

			// Setto il nuovo quaternione creando la matrice di rotazione attuale
			transform.setQuaternionRotation(totalRotationQuaternion);

			Utils::Vector3 debug = transform.getEulerRotation();

		}

		// Azzeramento forza risultante e momento risultante 
		resultantForce = resultantMomentum = Utils::Vector3::zero;
	}
}
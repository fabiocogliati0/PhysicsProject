#include "RigidBody.h"

#include "World.h"
#include "Collider.h"
#include "BoxCollider.h"

#include <cassert>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

namespace PhysicEngine
{

	RigidBody::RigidBody()
		: RigidBody(1.0f, PhysicMaterial(), BoxCollider(), Transform(), Utils::Vector3::zero)
	{
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider,
		staticBodyType isStatic
		)
		: RigidBody(mass, material, collider, Transform(), Utils::Vector3::zero)
	{
		staticBody = isStatic;
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider,
		const Transform& transform,
		staticBodyType isStatic
		)
		: RigidBody(mass, material, collider, transform, Utils::Vector3::zero)
	{
		staticBody = isStatic;
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider,
		const Transform& transform,
		const Utils::Vector3& velocity
		) 
		:	mass(mass), 
			material(material),
			transform(transform), 
			velocity(velocity), 
			staticBody(Non_Static_Body)
	{
		// TEMP
		this->momentum = Utils::Vector3::zero;
		this->angularMomentum = Utils::Vector3::zero;
		this->resultantForce = Utils::Vector3::zero;
		this->resultantMomentum = Utils::Vector3::zero;
		this->gravity = Utils::Vector3::zero;
		this->velocityOfGravity = Utils::Vector3::zero;
		this->angularVelocity = Utils::Vector3::zero;
		
		this->collider = collider.clone();
	}


	RigidBody::RigidBody(const RigidBody& other)
		:	mass(other.mass),
			material(other.material),
			transform(other.transform),
			staticBody(other.staticBody),
			velocity(other.velocity)
	{
		// TEMP
		this->momentum = other.momentum;
		this->angularMomentum = other.angularMomentum;
		this->gravity = other.gravity;
		this->velocityOfGravity = other.velocityOfGravity;

		assert(other.collider != nullptr);
		this->collider = other.collider->clone();
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

	RigidBody::~RigidBody()
	{
		delete collider;
		collider = nullptr;
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

	const Utils::Vector3 RigidBody::getAngularVelocity() const
	{
		return angularVelocity;
	}

	Utils::Vector3 RigidBody::getInertia() const
	{
		assert(collider != nullptr);
		return collider->getRawInertia() * mass;
	}

	float RigidBody::getArea() const
	{
		assert(collider != nullptr);
		return collider->getArea();
	}
	
	float RigidBody::getElasticity() const
	{
		return this->material.elasticity;
	}

	float RigidBody::getViscosity() const
	{
		return this->material.viscosity;
	}

	float RigidBody::getFriction() const
	{
		return this->material.friction;
	}

	RigidBody::staticBodyType RigidBody::getStaticBodyType() const
	{
		return staticBody;
	}

	void RigidBody::setVelocity(const Utils::Vector3 &velocityInput)
	{
		this->velocity = velocityInput;
	}

	// Applico una forza per un DT considerando come punto di applicazione il centro di massa
	void RigidBody::addForceDT(const Utils::Vector3& force)
	{
		if (this->getStaticBodyType() == Non_Static_Body)
		{
			this->addForceDT(Utils::Vector3::zero, force);
		}
	}

	// Applico una forza per un DT prendendo in considerazione il braccio della forza
	void RigidBody::addForceDT(const Utils::Vector3& point, const Utils::Vector3& force)
	{
		// Controllo se l'oggetto non è statico, altrimenti non applico nessuna forza
		if (this->getStaticBodyType()==Non_Static_Body)
		{
			// Sommo la forza ricevuta a quella che ho già
			this->resultantForce += force;
			if (point != Utils::Vector3::zero)
			{
				// Calcolo il momento risultante che mi servirà per ruotare l'oggetto
				Utils::Vector3 newResultantMomentum = point.cross(force);
				resultantMomentum += newResultantMomentum;
			}
		}
	}

	void RigidBody::updatePhyisic(float dt, const World& myWorld)
	{
		if (this->getStaticBodyType() == RigidBody::Non_Static_Body)
		{
			Utils::Quaternion newRotationQuaternion;
			Utils::Quaternion totalRotationQuaternion = transform.getRotationQuaternion();
			Utils::Matrix rotationMatrix = transform.getRotationMatrix();

			// Se la gravità è uguale l'ho già calcolata, quindi non la ricalcolo
			if (myWorld.getGravityForce() != gravity)
			{
				// calcolo nuovamente la velocità di gravità
				gravity = myWorld.getGravityForce();
				velocityOfGravity = gravity * dt;
			}

			// Moto translatorio
			if (resultantForce != Utils::Vector3::zero)
			{
				momentum = resultantForce * dt;
				velocity += momentum / mass;
			}

			// Aggiungo la componente velocità gravitazionale
			velocity += velocityOfGravity;

			// Moto rotatorio
			if (resultantMomentum != Utils::Vector3::zero)
			{
				angularMomentum += resultantMomentum * dt;
			}
			
			// Per risolvere problemi di inerzia, "raddrizzo" il mio
			// oggetto, altrimenti l'inerzia cambierebbe in base a come è disposto l'oggetto
			angularVelocity = rotationMatrix.RotateRelative(angularMomentum);

			// Ho effettivamente l'angularVelocity ora 
			angularVelocity.x /= this->getInertia().x;
			angularVelocity.y /= this->getInertia().y;
			angularVelocity.z /= this->getInertia().z;

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

				// Setto il nuovo quaternione creando la nuova matrice di rotazione
				transform.setQuaternionRotation(totalRotationQuaternion);
			}


			// ------ Calcolo la forza di attrito dell'aria
			Utils::Vector3 inverseVelocity;
			float drag = 0;
			float area = this->getArea();
			float modVelocity;

			// operazioni comuni
			inverseVelocity = velocity;
			inverseVelocity.invert();
			modVelocity = velocity.module();

			if (collider->getColliderType() == Collider::SphereColliderType)
				drag = 0.47f;
			else if (collider->getColliderType() == Collider::BoxColliderType)
				drag = 1.05f;

			// F = 1/2 * area * drag * airD * v^2 
			inverseVelocity = inverseVelocity * 0.5f * area * drag * myWorld.getAirDensity() * modVelocity;
			inverseVelocity /= mass; // Accelerazione
			inverseVelocity *= dt; // Velocità

			velocity += inverseVelocity;
			// ------ Fine calcolo attrito dell'aria


			// Ricalcolo la posizione, addizionando la posizione attuale alla nuova (velocity * dt)
			transform.setPosition(transform.getPosition() + (velocity * dt));


			// Azzeramento forza risultante e momento risultante 
			resultantForce = resultantMomentum = Utils::Vector3::zero;
		}
	}
}
#include "RigidBody.h"

#include "World.h"
#include "Collider.h"
#include "BoxCollider.h"

#include <assert.h>

namespace PhysicEngine
{

	RigidBody::RigidBody()
		: RigidBody(1.0f, PhysicMaterial(), BoxCollider(), Transform(), Utils::Vector3::zero, Utils::Vector3::zero)
	{
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider
		)
		: RigidBody(mass, material, collider, Transform(), Utils::Vector3::zero, Utils::Vector3::zero)
	{
	}

	RigidBody::RigidBody(float mass,
		const PhysicMaterial& material,
		const Collider& collider,
		const Transform& transform
		)
		: RigidBody(mass, material, collider, transform, Utils::Vector3::zero, Utils::Vector3::zero)
	{
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
			angularVelocity(angularVelocity)
	{
		this->collider = collider.clone();
	}


	RigidBody::RigidBody(const RigidBody& other)
		:	mass(other.mass),
			material(other.material),
			transform(other.transform),
			velocity(other.velocity),
			angularVelocity(other.angularVelocity)
	{

		assert(other.collider != nullptr);
		
		this->collider = other.collider->clone();
	}

	RigidBody::~RigidBody()
	{
		delete collider;
		collider = nullptr;
	}

	RigidBody& RigidBody::operator = (const RigidBody& other)
	{
		if (this != &other)
		{
			this->mass = mass;
			this->material = material;
			this->transform = transform;
			this->velocity = velocity;
			this->angularVelocity = angularVelocity;

			assert(other.collider != nullptr);
			this->collider = other.collider->clone();
		}

		return *this;
	}

	bool RigidBody::intersect(const RigidBody& other, Collision& o_collision) const
	{
		assert(collider != nullptr);
		return collider->intersect(*this, *(other.collider), other, o_collision);
	}

	const Utils::Vector3& RigidBody::getPosition() const
	{
		return transform.position;
	}

	//todo : manuele (vedi commenti nel .h)
	/*const Utils::Vector3& RigidBody::getRotation() const
	{
		return ???
	}*/

	const Utils::Vector3& RigidBody::getVelocity() const
	{
		return velocity;
	}

	const Utils::Vector3& RigidBody::getAngularVelocity() const
	{
		return angularVelocity;
	}

	const Utils::Vector3& RigidBody::getInertia() const
	{
		assert(collider != nullptr);
		return collider->getRawInertia() * mass;
	}

	float RigidBody::getVolume() const
	{
		assert(collider != nullptr);
		return collider->getVolume();
	}

	void RigidBody::addForce(const Utils::Vector3& force)
	{
		this->addForce(Utils::Vector3::zero, force);
	}

	void RigidBody::addForce(const Utils::Vector3& point, const Utils::Vector3& force)
	{
		// Sommo la forza ricevuta a quella che ho gi‡
		this->forzaRisultante += force;
		if (point != Utils::Vector3::zero)
		{
			// *** Calcolo il momento risultante che mi servir‡ per ruotare l'oggetto
			Utils::Vector3 mRisNew = point.cross(force);
			momentoRisultante += mRisNew;
			updateForce = true;
		}
	}

	void RigidBody::updatePhyisic(float dt, const World& myWorld)
	{
		// Se la gravit‡ Ë uguale l'ho gi‡ calcolata e mi evito una moltiplicazione
		if (myWorld.getGravityForce() != gravity)
		{
			// *** calcolo nuovamente la quantit‡ di moto della gravit‡
			// quantitadiMotoGravity = gravity * dt;
			velocityOfGravity = gravity * dt;
			gravity = myWorld.getGravityForce();
		}

		// Moto rettilineo uniforme
		if (updateForce)
		{
			quantitaDiMoto = forzaRisultante * dt; // Debug: forse +=
			velocity = quantitaDiMoto / mass;
			updateForce = false;
		}

		velocity += velocityOfGravity;
		transform.position += velocity * dt;

		// Moto angolare
		if (momentoRisultante != Utils::Vector3::zero)
			momentoAngolare = momentoRisultante * dt; // Debug: forse +=

		/* RuotaRelative(MRot, Mang, Vang); // Per risolvere problemi di inerzia, raddrizzo il mio
		// oggetto, altrimenti l'inerzia cambierebbe in base a come è disposto l'oggetto

		Vang[0] /= Inertia[0];
		Vang[1] /= Inertia[1];
		Vang[2] /= Inertia[2];

		q[0] = 1;
		q[1] = Vang[0] * dt / 2;
		q[2] = Vang[1] * dt / 2;
		q[3] = Vang[2] * dt / 2;

		NormalizzaQuaternione(q, q);
		MoltiplicaQuaternioni(Rot, q, Rot);
		NormalizzaQuaternione(Rot, Rot);

		RuotaAssolute(MRot, Vang, Vang); // Torno in assoluto per poi tornare in relativo
		MatriceDaQuaternione(Rot, MRot); */

	}
}
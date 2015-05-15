/*
* Progetto di Physics Programming
* Fabio Cogliati, Manuele Nerucci
*/

#include "PlaneCollider.h"

#include "Collider.h"
#include "Vector3.h"
#include <cassert>

namespace PhysicEngine
{

	PlaneCollider::PlaneCollider() 
		: PlaneCollider(0.0f, 1.0f, 0.0f, 0.0f, MajorLookDirection)
	{
	}

	PlaneCollider::PlaneCollider(float A, float B, float C, float D, lookDirections lookDirection)
		: A(A), B(B), C(C), D(D), lookDirection(lookDirection)
	{
		assert(A != 0.0f || B!=0.0f || C!= 0.0f);
	}

	PlaneCollider* PlaneCollider::clone() const
	{
		return new PlaneCollider(*this);
	}

	Collider::ColliderType PlaneCollider::getColliderType() const
	{
		return PlaneColliderType;
	}

	float PlaneCollider::getAFunctionCoefficient() const
	{
		return A;
	}

	float PlaneCollider::getBFunctionCoefficient() const
	{
		return B;
	}

	float PlaneCollider::getCFunctionCoefficient() const
	{
		return C;
	}

	float PlaneCollider::getDFunctionCoefficient() const
	{
		return D;
	}

	PlaneCollider::lookDirections PlaneCollider::getLookingDirection() const
	{
		return lookDirection;
	}

}
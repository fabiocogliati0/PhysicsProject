#include "BoxCollider.h"
#include "IntersectOperations.h"
#include "Collider.h"
class SphereCollider;

bool BoxCollider::intersect(const Collider& i_other, float o_intersection[3]) const{	//do double dispatch
	return i_other.intersectWho(*this, o_intersection);
}

bool BoxCollider::intersectWho(const SphereCollider& i_other, float o_intersection[3]) const{
	return IntersectOperations::intersect(*this, i_other, o_intersection);
}

bool BoxCollider::intersectWho(const BoxCollider& i_other, float o_intersection[3]) const{
	return IntersectOperations::intersect(*this, i_other, o_intersection);
	return true;
}
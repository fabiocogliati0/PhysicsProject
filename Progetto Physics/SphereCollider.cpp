#include "SphereCollider.h"
#include "IntersectOperations.h"
#include "Collider.h"
class BoxCollider;

bool SphereCollider::intersect(const Collider& i_other, float o_intersection[3]) const{	//do double dispatch
	return i_other.intersectWho(*this, o_intersection);
}

bool SphereCollider::intersectWho(const SphereCollider& i_other, float o_intersection[3]) const{
	return IntersectOperations::intersect(*this, i_other, o_intersection);
	return true;
}

bool SphereCollider::intersectWho(const BoxCollider& i_other, float o_intersection[3]) const{
	return IntersectOperations::intersect(*this, i_other, o_intersection);
	return true;
}
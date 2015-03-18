#pragma once

#include "Collider.h"
class BoxCollider;

class SphereCollider : public Collider{

public:

	bool intersect(const Collider& i_other, float o_intersection[3]) const;	//do double dispatch
	
private:

	bool intersectWho(const SphereCollider& i_other, float o_intersection[3]) const;
	bool intersectWho(const BoxCollider& i_other, float o_intersection[3]) const;

	float radius;

};
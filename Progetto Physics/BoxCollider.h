#pragma once

#include "Collider.h"
class SphereCollider;

class BoxCollider : public Collider{

public:

	bool intersect(const Collider& i_other, float o_intersection[3]) const;	//do double dispatch

protected:

	bool intersectWho(const SphereCollider& i_other, float o_intersection[3]) const;
	bool intersectWho(const BoxCollider& i_other, float o_intersection[3])const;

	float sizeX;
	float sizeY;
	float sizeZ;

};
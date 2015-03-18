#pragma once

class SphereCollider;
class BoxCollider;

class Collider{

public:

	virtual bool intersect(const Collider& i_other, float o_intersection[3]) const = 0;
	virtual bool intersectWho(const SphereCollider& i_other,	float o_intersection[3]) const = 0;
	virtual bool intersectWho(const BoxCollider& i_other,	float o_intersection[3]) const = 0;

private:

	float offset[3];

};

#include <iostream>

#include "World.h"
#include "BoxCollider.h"
#include "SphereCollider.h"

int main(){

	BoxCollider a;
	SphereCollider b;
	float res[3];

	a.intersect(b,res);

	std::getchar();
}
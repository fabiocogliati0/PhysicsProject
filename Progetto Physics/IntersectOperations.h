#pragma once

#include "SphereCollider.h"
#include "BoxCollider.h"

#include <iostream>

class IntersectOperations{

public:

	template <class T, class U> static bool  intersect(const T& i_coll1, const U& i_coll2, float o_intersection[3])
	{
		std::cout << "switch" << std::endl;
		return intersect(i_coll2,i_coll1,o_intersection);
	}

	template<> static bool intersect<SphereCollider, SphereCollider>
		(const SphereCollider& i_coll1, const SphereCollider& i_coll2, float o_intersection[3])
	{
		std::cout << "sphere-sphere" << std::endl;
		return true;	//implement sphere-sphere intersection
	}

	template<> static bool intersect<BoxCollider, SphereCollider>
		(const BoxCollider& i_coll1, const SphereCollider& i_coll2, float o_intersection[3])
	{
		std::cout << "sphere-box" << std::endl;
		return true;	//implement sphere-box intersection
	}

	template<> static bool intersect<BoxCollider, BoxCollider>
		(const BoxCollider& i_coll1, const BoxCollider& i_coll2, float o_intersection[3])
	{
		std::cout << "box-box" << std::endl;
		return true;	//implement box-box intersection
	}

private:
	
	IntersectOperations();

};
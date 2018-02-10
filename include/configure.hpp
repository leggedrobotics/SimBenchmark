//
// Created by jhwangbo on 18. 1. 2.
//

#ifndef BENCHMARK_CONFIGURE_HPP
#define BENCHMARK_CONFIGURE_HPP

typedef int CollisionGroupType;

// TODO ODE
//typedef unsigned long CollisionGroupType;

namespace benchmark {
enum ObjectType { SPHERE, BOX, CYLINDER, CONE, CAPSULE, CONVEXMESH, HALFSPACE, COMPOUND, HEIGHTMAP, ARTICULATED_SYSTEM };
}

#endif //BENCHMARK_CONFIGURE_HPP

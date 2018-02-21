//
// Created by kangd on 13.02.18.
//

#ifndef ODESIM_SPHERE_HPP
#define ODESIM_SPHERE_HPP

#include "SingleBodyObject.hpp"

namespace ode_sim {
namespace object {

class Sphere: public SingleBodyObject {

 public:
  Sphere(double radius,
         double mass,
         dWorldID worldID,
         dSpaceID spaceID,
         benchmark::CollisionGroupType collisionGroup,
         benchmark::CollisionGroupType collisionMask);

};

} // object
} // ode_sim

#endif //BENCHMARK_SPHERE_HPP

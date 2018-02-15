//
// Created by kangd on 15.02.18.
//

#ifndef BENCHMARK_CAPSULE_HPP
#define BENCHMARK_CAPSULE_HPP

#include <Configure.hpp>
#include "SingleBodyObject.hpp"

namespace ode_sim {
namespace object {

class Capsule: public SingleBodyObject {

 public:
  Capsule(double radius,
          double height,
          double mass,
          dWorldID worldID,
          dSpaceID spaceID,
          CollisionGroupType collisionGroup,
          CollisionGroupType collisionMask);

};

} // object
} // ode_sim

#endif //BENCHMARK_CAPSULE_HPP

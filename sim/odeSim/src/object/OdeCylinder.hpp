//
// Created by kangd on 14.04.18.
//

#ifndef BENCHMARK_ODECYLINDER_HPP
#define BENCHMARK_ODECYLINDER_HPP

#include <common/Configure.hpp>
#include "OdeSingleBodyObject.hpp"

namespace ode_sim {
namespace object {

class OdeCylinder: public OdeSingleBodyObject {

 public:
  OdeCylinder(double radius,
              double height,
              double mass,
              dWorldID worldID,
              dSpaceID spaceID,
              benchmark::CollisionGroupType collisionGroup,
              benchmark::CollisionGroupType collisionMask);

};

}
}

#endif //BENCHMARK_ODECYLINDER_HPP

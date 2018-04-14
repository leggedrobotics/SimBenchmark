//
// Created by kangd on 13.02.18.
//

#ifndef ODESIM_SPHERE_HPP
#define ODESIM_SPHERE_HPP

#include "common/Configure.hpp"
#include "OdeSingleBodyObject.hpp"

namespace ode_sim {
namespace object {

class OdeSphere: public OdeSingleBodyObject {

 public:
  OdeSphere(double radius,
            double mass,
            dWorldID worldID,
            dSpaceID spaceID,
            benchmark::CollisionGroupType collisionGroup,
            benchmark::CollisionGroupType collisionMask);

};

} // object
} // ode_sim

#endif //BENCHMARK_SPHERE_HPP

//
// Created by kangd on 15.02.18.
//

#ifndef ODESIM_CAPSULE_HPP
#define ODESIM_CAPSULE_HPP

#include "common/Configure.hpp"
#include "OdeSingleBodyObject.hpp"

namespace ode_sim {
namespace object {

class OdeCapsule: public OdeSingleBodyObject {

 public:
  OdeCapsule(double radius,
             double height,
             double mass,
             dWorldID worldID,
             dSpaceID spaceID,
             benchmark::CollisionGroupType collisionGroup,
             benchmark::CollisionGroupType collisionMask);

};

} // object
} // ode_sim

#endif //ODESIM_CAPSULE_HPP

//
// Created by kangd on 11.02.18.
//

#ifndef ODESIM_BOX_HPP
#define ODESIM_BOX_HPP

#include "common/Configure.hpp"
#include "OdeSingleBodyObject.hpp"

namespace ode_sim {
namespace object {

class OdeBox: public OdeSingleBodyObject {

 public:
  OdeBox(double xlength,
         double ylength,
         double zlength,
         double mass,
         dWorldID worldID,
         dSpaceID spaceID,
         benchmark::CollisionGroupType collisionGroup,
         benchmark::CollisionGroupType collisionMask);
  virtual ~OdeBox();

};

} // object
} // ode_sim

#endif //ODESIM_BOX_HPP

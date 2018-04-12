//
// Created by kangd on 15.02.18.
//

#ifndef BULLETSIM_CAPSULE_HPP
#define BULLETSIM_CAPSULE_HPP

#include "BtSingleBodyObject.hpp"

namespace bullet_sim {
namespace object {

class BtCapsule: public BtSingleBodyObject {

 public:
  BtCapsule(double radius, double height, double mass);

};

} // object
} // bullet_sim

#endif //BULLETSIM_CAPSULE_HPP

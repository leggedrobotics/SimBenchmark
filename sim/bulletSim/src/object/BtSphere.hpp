//
// Created by kangd on 13.02.18.
//

#ifndef BULLETSIM_SPHERE_HPP
#define BULLETSIM_SPHERE_HPP

#include "BtSingleBodyObject.hpp"

namespace bullet_sim {
namespace object {

class BtSphere: public BtSingleBodyObject {

 public:
  BtSphere(double radius, double mass);

};

} // object
} // bullet_sim

#endif //BULLETSIM_SPHERE_HPP

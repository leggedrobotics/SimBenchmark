//
// Created by kangd on 01.04.18.
//

#ifndef BULLETSIM_CYLINDER_HPP
#define BULLETSIM_CYLINDER_HPP

#include "BtSingleBodyObject.hpp"

namespace bullet_sim {
namespace object {

class BtCylinder: public BtSingleBodyObject {

 public:
  BtCylinder(double radius, double height, double mass);

};

} // object
} // bullet_sim

#endif //BULLETSIM_CYLINDER_HPP

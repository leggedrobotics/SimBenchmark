//
// Created by kangd on 01.04.18.
//

#ifndef BULLETSIM_CYLINDER_HPP
#define BULLETSIM_CYLINDER_HPP

#include "SingleBodyObject.hpp"

namespace bullet_sim {
namespace object {

class Cylinder: public SingleBodyObject {

 public:
  Cylinder(float radius, float height, float mass);

};

} // object
} // bullet_sim

#endif //BULLETSIM_CYLINDER_HPP

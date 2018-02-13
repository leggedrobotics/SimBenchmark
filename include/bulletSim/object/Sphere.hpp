//
// Created by kangd on 13.02.18.
//

#ifndef BULLETSIM_SPHERE_HPP
#define BULLETSIM_SPHERE_HPP

#include "SingleBodyObject.hpp"

namespace bullet_sim {
namespace object {

class Sphere: public SingleBodyObject {

 public:
  Sphere(double radius, double mass);

};

} // object
} // bullet_sim

#endif //BULLETSIM_SPHERE_HPP

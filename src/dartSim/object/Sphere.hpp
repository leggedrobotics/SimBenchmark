//
// Created by kangd on 19.03.18.
//

#ifndef DARTSIM_SPHERE_HPP
#define DARTSIM_SPHERE_HPP

#include "SingleBodyObject.hpp"

namespace dart_sim {
namespace object {

class Sphere: public SingleBodyObject {

 public:
  Sphere(double radius, double mass);

};

}
}

#endif //DARTSIM_SPHERE_HPP

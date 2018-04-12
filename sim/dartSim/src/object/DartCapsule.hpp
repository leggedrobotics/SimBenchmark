//
// Created by kangd on 19.03.18.
//

#ifndef DARTSIM_CAPSULE_HPP
#define DARTSIM_CAPSULE_HPP

#include "DartSingleBodyObject.hpp"

namespace dart_sim {
namespace object {

class DartCapsule: public DartSingleBodyObject {

 public:
  DartCapsule(double radius, double height, double mass, int id);

};

} // object
} // dart_sim

#endif //DARTSIM_CAPSULE_HPP

//
// Created by kangd on 19.02.18.
//

#ifndef MUJOCOSIM_CAPSULE_HPP
#define MUJOCOSIM_CAPSULE_HPP

#include "SingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

class Capsule: public SingleBodyObject {
 public:
  Capsule(double radius, double height, mjData *data, int objectID);
};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_CAPSULE_HPP

//
// Created by kangd on 19.02.18.
//

#ifndef MUJOCOSIM_CAPSULE_HPP
#define MUJOCOSIM_CAPSULE_HPP

#include "MjcSingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

class MjcCapsule: public MjcSingleBodyObject {

 public:
  MjcCapsule(double radius, double height, mjData *data, mjModel *model, int bodyId, int geomId);

 private:
  double radius_;
  double height_;
};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_CAPSULE_HPP

//
// Created by kangd on 19.02.18.
//

#ifndef MUJOCOSIM_BOX_HPP
#define MUJOCOSIM_BOX_HPP

#include "MjcSingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

class MjcBox: public MjcSingleBodyObject {

 public:
  MjcBox(double xLength, double yLength, double zLength, mjData *data, mjModel *model, int bodyId, int geomId);

 private:
  double xLength_;
  double yLength_;
  double zLength_;
};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_BOX_HPP

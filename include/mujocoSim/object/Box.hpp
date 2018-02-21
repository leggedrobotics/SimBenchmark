//
// Created by kangd on 19.02.18.
//

#ifndef MUJOCOSIM_BOX_HPP
#define MUJOCOSIM_BOX_HPP

#include "SingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

class Box: public SingleBodyObject {

 public:
  Box(double xLength, double yLength, double zLength, mjData *data, mjModel *model, int objectID);

};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_BOX_HPP

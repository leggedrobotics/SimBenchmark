//
// Created by kangd on 19.02.18.
//

#ifndef MUJOCOSIM_CHECKERBOARD_HPP
#define MUJOCOSIM_CHECKERBOARD_HPP

#include "SingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

class CheckerBoard: public SingleBodyObject {

 public:
  CheckerBoard(double xLength, double yLength, mjData *data, int objectID);

};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_CHECKERBOARD_HPP

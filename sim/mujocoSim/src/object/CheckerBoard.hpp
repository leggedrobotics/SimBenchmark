//
// Created by kangd on 19.02.18.
//

#ifndef MUJOCOSIM_CHECKERBOARD_HPP
#define MUJOCOSIM_CHECKERBOARD_HPP

#include "common/interface/CheckerboardInterface.hpp"
#include "SingleBodyObject.hpp"

namespace bo = benchmark::object;

namespace mujoco_sim {
namespace object {

class CheckerBoard: public SingleBodyObject,
                    public bo::CheckerboardInterface {

 public:
  CheckerBoard(double xLength,
               double yLength,
               mjData *data,
               mjModel *model,
               int bodyId,
               int geomId);

 private:
  double xLength_;
  double yLength_;

};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_CHECKERBOARD_HPP

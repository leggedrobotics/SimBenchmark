//
// Created by kangd on 01.04.18.
//

#ifndef MUJOCOSIM_CYLINDER_HPP
#define MUJOCOSIM_CYLINDER_HPP

#include "MjcSingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

class MjcCylinder: public MjcSingleBodyObject {

 public:
  MjcCylinder(double radius,
              double height,
              mjData *data,
              mjModel *model,
              int bodyId,
              int geomId);

 private:
  double radius_;
  double height_;

};

}
}

#endif //MUJOCOSIM_CYLINDER_HPP

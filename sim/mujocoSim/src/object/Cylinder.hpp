//
// Created by kangd on 01.04.18.
//

#ifndef MUJOCOSIM_CYLINDER_HPP
#define MUJOCOSIM_CYLINDER_HPP

#include "SingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

class Cylinder: public SingleBodyObject {

 public:
  Cylinder(double radius,
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

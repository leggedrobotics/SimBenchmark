//
// Created by kangd on 19.02.18.
//

#ifndef MUJOCOSIM_SPHERE_HPP
#define MUJOCOSIM_SPHERE_HPP

#include "SingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

class Sphere: public SingleBodyObject {

 public:
  Sphere(double radius, double mass, mjData *data, mjModel *model, int objectID);

 private:
  double radius_;
  double mass_;

};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_SPHERE_HPP

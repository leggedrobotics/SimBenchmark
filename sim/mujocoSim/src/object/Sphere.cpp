//
// Created by kangd on 19.02.18.
//

#include "Sphere.hpp"

mujoco_sim::object::Sphere::Sphere(double radius, double mass, mjData *data, mjModel *model, int objectID)
    : SingleBodyObject(data, model, objectID),
      radius_(radius),
      mass_(mass) {}

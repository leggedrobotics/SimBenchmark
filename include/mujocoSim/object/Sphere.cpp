//
// Created by kangd on 19.02.18.
//

#include "Sphere.hpp"

mujoco_sim::object::Sphere::Sphere(double radius, double mass, mjData *data, int objectID)
    : SingleBodyObject(data, objectID),
      radius_(radius),
      mass_(mass) {}

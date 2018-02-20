//
// Created by kangd on 19.02.18.
//

#include "Capsule.hpp"

mujoco_sim::object::Capsule::Capsule(double radius, double height, mjData *data, int objectID)
    : SingleBodyObject(data, objectID) {}

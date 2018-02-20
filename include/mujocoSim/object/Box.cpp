//
// Created by kangd on 19.02.18.
//

#include "Box.hpp"

mujoco_sim::object::Box::Box(double xLength, double yLength, double zLength, mjData *data, int objectID)
    : SingleBodyObject(data, objectID) {}

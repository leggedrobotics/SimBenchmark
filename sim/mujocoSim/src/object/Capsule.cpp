//
// Created by kangd on 19.02.18.
//

#include "Capsule.hpp"

mujoco_sim::object::Capsule::Capsule(double radius, double height, mjData *data, mjModel *model, int bodyId, int geomId)
    : SingleBodyObject(data, model, bodyId, geomId), radius_(radius), height_(height) {}

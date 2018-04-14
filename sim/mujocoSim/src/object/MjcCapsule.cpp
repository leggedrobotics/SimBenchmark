//
// Created by kangd on 19.02.18.
//

#include "MjcCapsule.hpp"

mujoco_sim::object::MjcCapsule::MjcCapsule(double radius, double height, mjData *data, mjModel *model, int bodyId, int geomId)
    : MjcSingleBodyObject(data, model, bodyId, geomId), radius_(radius), height_(height) {}

//
// Created by kangd on 01.04.18.
//

#include "Cylinder.hpp"

mujoco_sim::object::Cylinder::Cylinder(double radius,
                                       double height,
                                       mjData *data,
                                       mjModel *model,
                                       int bodyId,
                                       int geomId)
    : SingleBodyObject(data, model, bodyId, geomId), radius_(radius), height_(height) {}

//
// Created by kangd on 19.02.18.
//

#include "Box.hpp"

mujoco_sim::object::Box::Box(double xLength,
                             double yLength,
                             double zLength,
                             mjData *data,
                             mjModel *model,
                             int bodyId,
                             int geomId)
    : SingleBodyObject(data, model, bodyId, geomId), xLength_(xLength), yLength_(yLength), zLength_(zLength) {}

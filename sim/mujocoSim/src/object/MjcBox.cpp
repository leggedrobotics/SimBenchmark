//
// Created by kangd on 19.02.18.
//

#include "MjcBox.hpp"

mujoco_sim::object::MjcBox::MjcBox(double xLength,
                             double yLength,
                             double zLength,
                             mjData *data,
                             mjModel *model,
                             int bodyId,
                             int geomId)
    : MjcSingleBodyObject(data, model, bodyId, geomId), xLength_(xLength), yLength_(yLength), zLength_(zLength) {}

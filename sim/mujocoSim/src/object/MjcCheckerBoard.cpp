//
// Created by kangd on 19.02.18.
//

#include "MjcCheckerBoard.hpp"
mujoco_sim::object::MjcCheckerBoard::MjcCheckerBoard(double xLength,
                                               double yLength,
                                               mjData *data,
                                               mjModel *model,
                                               int bodyId,
                                               int geomId)
    : MjcSingleBodyObject(data, model, bodyId, geomId), xLength_(xLength), yLength_(yLength) {}

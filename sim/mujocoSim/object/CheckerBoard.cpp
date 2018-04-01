//
// Created by kangd on 19.02.18.
//

#include "CheckerBoard.hpp"
mujoco_sim::object::CheckerBoard::CheckerBoard(double xLength, double yLength, mjData *data, mjModel *model, int objectID)
    : SingleBodyObject(data, model, objectID) {}

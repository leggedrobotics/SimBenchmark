//
// Created by kangd on 19.02.18.
//

#include "Sphere.hpp"

mujoco_sim::object::Sphere::Sphere(double radius,
                                   double mass,
                                   mjData *data,
                                   mjModel *model,
                                   int bodyId,
                                   int geomId)
    : SingleBodyObject(data, model, bodyId, geomId), radius_(radius), mass_(mass) {}

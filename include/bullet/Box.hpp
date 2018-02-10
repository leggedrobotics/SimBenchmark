//
// Created by kangd on 09.02.18.
//

#ifndef BENCHMARK_BULLETBOX_HPP
#define BENCHMARK_BULLETBOX_HPP

#include <btBulletCollisionCommon.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

namespace bullet_sim {
namespace object {

class Box {

 public:
  Box(double xlength, double ylength, double zlength, double mass);

 private:
  btBoxShape *shape_;
  btRigidBody *rigidBody_;
  btMotionState *motionState_;

};

} // object
} // bullet_sim

#endif //BENCHMARK_BULLETBOX_HPP

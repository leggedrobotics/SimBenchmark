//
// Created by kangd on 09.02.18.
//

#include <bullet/Box.hpp>

bullet_sim::object::Box::Box(double xlength, double ylength, double zlength, double mass) {
  shape_ = new btBoxShape(btVector3(.5 * xlength,
                                    .5 * ylength,
                                    .5 * zlength));

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(btVector3(0, 0, 0));
  motionState_ = new btDefaultMotionState(transform);

  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState_, shape_);
  rigidBody_ = new btRigidBody(rbInfo);
}

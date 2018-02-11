//
// Created by kangd on 10.02.18.
//

#include "bulletSim/object/CheckerBoard.hpp"

bullet_sim::object::CheckerBoard::CheckerBoard() {

  // collision shape
  collisionShape_ = new btStaticPlaneShape(btVector3(0, 0, 1), 0);

  // position and orientation
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(btVector3(0, 0, 0));
  motionState_ = new btDefaultMotionState(transform);

  // rigid body
  /// mass of ground is zero (inf mass)
  btRigidBody::btRigidBodyConstructionInfo rbInfo(0, motionState_, collisionShape_);
  rigidBody_ = new btRigidBody(rbInfo);
}

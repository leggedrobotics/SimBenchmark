//
// Created by kangd on 10.02.18.
//

#include "BtCheckerBoard.hpp"

bullet_sim::object::BtCheckerBoard::BtCheckerBoard(double xLength, double yLength, bo::CheckerboardShape shape)
    : BtSingleBodyObject(0) {

  // position and orientation
  btTransform transform;
  transform.setIdentity();

  // collision shape
  if(shape == bo::PLANE_SHAPE) {
    // plane shape
    collisionShape_ = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
  transform.setOrigin(btVector3(0, 0, 0));
  } else {
    // box shape
    collisionShape_ = new btBoxShape(btVector3(btScalar(xLength * 0.5),btScalar(yLength * 0.5),btScalar(10.)));
    transform.setOrigin(btVector3(0, 0, -10));
  }

  motionState_ = new btDefaultMotionState(transform);

  // rigid body
  /// mass of ground is zero (inf mass)
  btRigidBody::btRigidBodyConstructionInfo rbInfo(0, motionState_, collisionShape_);
  rigidBody_ = new btRigidBody(rbInfo);
}

//
// Created by kangd on 10.02.18.
//

#include "CheckerBoard.hpp"

bullet_sim::object::CheckerBoard::CheckerBoard(double xLength, double yLength) : SingleBodyObject(0) {

  // collision shape
  collisionShape_ = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
//  collisionShape_ = new btBoxShape(btVector3(btScalar(xLength * 0.5),btScalar(yLength * 0.5),btScalar(10.)));

  // position and orientation
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(btVector3(0, 0, 0));
//  transform.setOrigin(btVector3(0, 0, -10));
  motionState_ = new btDefaultMotionState(transform);

  // rigid body
  /// mass of ground is zero (inf mass)
  btRigidBody::btRigidBodyConstructionInfo rbInfo(0, motionState_, collisionShape_);
  rigidBody_ = new btRigidBody(rbInfo);
}

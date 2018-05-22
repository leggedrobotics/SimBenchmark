//
// Created by kangd on 01.04.18.
//

#include "BtCylinder.hpp"

namespace bullet_sim {
namespace object {

BtCylinder::BtCylinder(double radius, double height, double mass): BtSingleBodyObject(mass) {

  // collision shape
  collisionShape_ = new btCylinderShapeZ(btVector3(radius,
                                                   radius,
                                                   height * 0.5));

  // position and orientation
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(btVector3(0, 0, 0));
  motionState_ = new btDefaultMotionState(transform);

  // local inertia
  btVector3 localInertia(0, 0, 0);
  collisionShape_->calculateLocalInertia(mass, localInertia);

  // rigid body
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState_, collisionShape_, localInertia);
  rigidBody_ = new btRigidBody(rbInfo);
  rigidBody_->setFlags(BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY);
  rigidBody_->setSleepingThresholds(0, 0);
}


} // object
} // bullet_sim
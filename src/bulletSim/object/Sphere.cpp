//
// Created by kangd on 13.02.18.
//

#include "Sphere.hpp"

bullet_sim::object::Sphere::Sphere(double radius, double mass) : SingleBodyObject(mass) {

// collision shape
  collisionShape_ = new btSphereShape(radius);

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
}

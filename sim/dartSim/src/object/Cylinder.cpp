//
// Created by kangd on 04.04.18.
//

#include "Cylinder.hpp"

dart_sim::object::Cylinder::Cylinder(double radius, double height, double mass): SingleBodyObject(mass) {
  skeletonPtr_ = dart::dynamics::Skeleton::create();
  shapePtr_ = dart::dynamics::ShapePtr(new dart::dynamics::CylinderShape(radius, height));

  dart::dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = "cylinder_link";
  bodyProp.mInertia.setMass(mass);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  dart::dynamics::FreeJoint::Properties jointProp;
  jointProp.mName = "box_joint";
  jointProp.mT_ParentBodyToJoint = T;

  auto pair = skeletonPtr_->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProp, bodyProp);
  auto shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shapePtr_);

  bodyPtr_ = pair.second;
}

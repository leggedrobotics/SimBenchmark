//
// Created by kangd on 19.03.18.
//

#include "Box.hpp"

namespace dart_sim {
namespace object {

Box::Box(double xlength, double ylength, double zlength, double mass): SingleBodyObject(mass) {
  skeletonPtr_ = dart::dynamics::Skeleton::create();
  shapePtr_ = dart::dynamics::ShapePtr(new dart::dynamics::BoxShape(
      Eigen::Vector3d(xlength, ylength, zlength)));

  dart::dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = "box_link";
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
}

} // object
} // dart_sim
//
// Created by kangd on 19.03.18.
//

#include "CheckerBoard.hpp"
dart_sim::object::CheckerBoard::CheckerBoard(double xLength, double yLength): SingleBodyObject(0) {

  skeletonPtr_ = dart::dynamics::Skeleton::create();
  shapePtr_ = dart::dynamics::ShapePtr(new dart::dynamics::PlaneShape(
      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0)));

  dart::dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = "ground_link";

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  dart::dynamics::FreeJoint::Properties jointProp;
  jointProp.mName = "ground_joint";
  jointProp.mT_ParentBodyToJoint = T;

  auto pair = skeletonPtr_->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProp, bodyProp);
  auto shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shapePtr_);

}

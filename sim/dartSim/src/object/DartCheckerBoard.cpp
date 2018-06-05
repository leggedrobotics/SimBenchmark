//
// Created by kangd on 19.03.18.
//

#include "DartCheckerBoard.hpp"

dart_sim::object::DartCheckerBoard::DartCheckerBoard(double xLength, double yLength, benchmark::object::CheckerboardShape shape, int id)
    : DartSingleBodyObject(0, id) {

  // props
  dart::dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = "ground_link";

  dart::dynamics::FreeJoint::Properties jointProp;
  jointProp.mName = "ground_joint";
  jointProp.mDampingCoefficients[0] = 0;
  jointProp.mDampingCoefficients[1] = 0;
  jointProp.mDampingCoefficients[2] = 0;
  jointProp.mDampingCoefficients[3] = 0;
  jointProp.mDampingCoefficients[4] = 0;
  jointProp.mDampingCoefficients[5] = 0;

  // body
  auto pair = skeletonPtr_->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
      nullptr, jointProp, bodyProp);
  bodyPtr_ = pair.second;

  // shape
  if(shape == benchmark::object::BOX_SHAPE) {
    shapePtr_ = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(xLength, yLength, 10));
  }
  else if(shape == benchmark::object::PLANE_SHAPE) {
    shapePtr_ = std::make_shared<dart::dynamics::PlaneShape>(Eigen::Vector3d(0, 0, 1), 0);
  }

  auto shapeNode = bodyPtr_->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shapePtr_);

  // position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

  if(shape == benchmark::object::BOX_SHAPE)
    tf.translation() = Eigen::Vector3d(0, 0, -5);

  bodyPtr_->getParentJoint()->setTransformFromParentBodyNode(tf);
}

void dart_sim::object::DartCheckerBoard::getPosition_W(benchmark::Vec<3> &pos_w) {
  pos_w = {0, 0, 0};
}
void dart_sim::object::DartCheckerBoard::getQuaternion(benchmark::Vec<4> &quat) {
  quat = {1, 0, 0, 0};
}


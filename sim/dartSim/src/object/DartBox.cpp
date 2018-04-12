//
// Created by kangd on 19.03.18.
//

#include "DartBox.hpp"

namespace dart_sim {
namespace object {

DartBox::DartBox(double xlength, double ylength, double zlength, double mass, int id) : DartSingleBodyObject(mass, id) {

  // skeleton
  skeletonPtr_ = dart::dynamics::Skeleton::create();

  // props
  dart::dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = std::to_string(id) + "_link";

  dart::dynamics::FreeJoint::Properties jointProp;
  jointProp.mName = std::to_string(id) + "_joint";

  // body
  auto pair = skeletonPtr_->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProp, bodyProp);
  bodyPtr_ = pair.second;

  // shape
  shapePtr_ = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(xlength, ylength, zlength));
  auto shapeNode = bodyPtr_->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shapePtr_);

  // inertia
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shapePtr_->computeInertia(mass));
  bodyPtr_->setInertia(inertia);
}

} // object
} // dart_sim
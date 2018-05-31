//
// Created by kangd on 19.03.18.
//

#include "DartCapsule.hpp"

namespace dart_sim {
namespace object {

DartCapsule::DartCapsule(double radius, double height, double mass, int id) : DartSingleBodyObject(mass, id) {

  // props
  dart::dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = std::to_string(id) + "_link";

  dart::dynamics::FreeJoint::Properties jointProp;
  jointProp.mName = std::to_string(id) + "_joint";
  jointProp.mDampingCoefficients[0] = 0;
  jointProp.mDampingCoefficients[1] = 0;
  jointProp.mDampingCoefficients[2] = 0;
  jointProp.mDampingCoefficients[3] = 0;
  jointProp.mDampingCoefficients[4] = 0;
  jointProp.mDampingCoefficients[5] = 0;

  // body
  auto pair = skeletonPtr_->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProp, bodyProp);
  bodyPtr_ = pair.second;

  // shape
  shapePtr_ = std::make_shared<dart::dynamics::CapsuleShape>(
      radius, height);
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
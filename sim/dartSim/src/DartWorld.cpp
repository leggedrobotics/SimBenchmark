//
// Created by kangd on 16.03.18.
//

#include "DartWorld.hpp"

namespace dart_sim {

DartWorld::DartWorld() : dynamicsWorld_(std::make_shared<dart::simulation::World>()) {
  dynamicsWorld_->setGravity(gravity_);
}

DartWorld::~DartWorld() {
  // remove objects
  for (auto *ob: objectList_)
    delete ob;
}

object::DartSphere *DartWorld::addSphere(double radius,
                                 double mass,
                                 benchmark::CollisionGroupType collisionGroup,
                                 benchmark::CollisionGroupType collisionMask) {
  auto *sphere = new dart_sim::object::DartSphere(radius, mass, objectList_.size());
  dynamicsWorld_->addSkeleton(sphere->getSkeletonPtr());
  objectList_.push_back(sphere);
  return sphere;
}

object::DartBox *DartWorld::addBox(double xLength,
                           double yLength,
                           double zLength,
                           double mass,
                           benchmark::CollisionGroupType collisionGroup,
                           benchmark::CollisionGroupType collisionMask) {
  auto *box = new dart_sim::object::DartBox(xLength, yLength, zLength, mass, objectList_.size());
  dynamicsWorld_->addSkeleton(box->getSkeletonPtr());
  objectList_.push_back(box);
  return box;
}

object::DartCapsule *DartWorld::addCapsule(double radius,
                                   double height,
                                   double mass,
                                   benchmark::CollisionGroupType collisionGroup,
                                   benchmark::CollisionGroupType collisionMask) {
  auto *capsule = new dart_sim::object::DartCapsule(radius, height, mass, objectList_.size());
  dynamicsWorld_->addSkeleton(capsule->getSkeletonPtr());
  objectList_.push_back(capsule);
  return capsule;
}

object::DartCheckerBoard *DartWorld::addCheckerboard(double gridSize,
                                             double xLength,
                                             double yLength,
                                             double reflectanceI,
                                             bo::CheckerboardShape shape,
                                             benchmark::CollisionGroupType collisionGroup,
                                             benchmark::CollisionGroupType collisionMask) {
  auto *checkerBoard = new dart_sim::object::DartCheckerBoard(xLength, yLength, shape, objectList_.size());
  dynamicsWorld_->addSkeleton(checkerBoard->getSkeletonPtr());
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

object::DartCylinder *DartWorld::addCylinder(double radius,
                                     double height,
                                     double mass,
                                     benchmark::CollisionGroupType collisionGroup,
                                     benchmark::CollisionGroupType collisionMask) {
  auto *cylinder = new dart_sim::object::DartCylinder(radius, height, mass, objectList_.size());
  dynamicsWorld_->addSkeleton(cylinder->getSkeletonPtr());
  objectList_.push_back(cylinder);
  return cylinder;
}

object::DartArticulatedSystem *DartWorld::addArticulatedSystem(std::string urdfPath,
                                                       benchmark::CollisionGroupType collisionGroup,
                                                       benchmark::CollisionGroupType collisionMask) {
  auto *robot = new dart_sim::object::DartArticulatedSystem(urdfPath);
  dynamicsWorld_->addSkeleton(robot->getSkeletonPtr());
  objectList_.push_back(robot);
  return robot;
}

void DartWorld::setGravity(const benchmark::Vec<3> &gravity) {
  gravity_[0] = gravity[0];
  gravity_[1] = gravity[1];
  gravity_[2] = gravity[2];
  dynamicsWorld_->setGravity(gravity_);
}

void DartWorld::integrate() {
  dynamicsWorld_->step();
}

void DartWorld::setTimeStep(double timeStep) {
  timeStep_ = timeStep;
  dynamicsWorld_->setTimeStep(timeStep);
}

void DartWorld::integrate(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate() instead")
}

int DartWorld::getNumObject() {
  return objectList_.size();
}

} // dart_sim
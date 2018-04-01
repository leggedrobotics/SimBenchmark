//
// Created by kangd on 16.03.18.
//

#include "World.hpp"

namespace dart_sim {

World::World() : dynamicsWorld_(std::make_shared<dart::simulation::World>()) {
  dynamicsWorld_->setGravity(gravity_);
}

World::~World() {
  // remove objects
  for (auto *ob: objectList_)
    delete ob;
}

object::Sphere *World::addSphere(double radius,
                                 double mass,
                                 benchmark::CollisionGroupType collisionGroup,
                                 benchmark::CollisionGroupType collisionMask) {
  dart_sim::object::Sphere *sphere = new dart_sim::object::Sphere(radius, mass);
  dynamicsWorld_->addSkeleton(sphere->getSkeletonPtr());
  objectList_.push_back(sphere);
  return sphere;
}

object::Box *World::addBox(double xLength,
                           double yLength,
                           double zLength,
                           double mass,
                           benchmark::CollisionGroupType collisionGroup,
                           benchmark::CollisionGroupType collisionMask) {
  dart_sim::object::Box *box = new dart_sim::object::Box(xLength, yLength, zLength, mass);
  dynamicsWorld_->addSkeleton(box->getSkeletonPtr());
  objectList_.push_back(box);
  return box;
}

object::Capsule *World::addCapsule(double radius,
                                   double height,
                                   double mass,
                                   benchmark::CollisionGroupType collisionGroup,
                                   benchmark::CollisionGroupType collisionMask) {
  dart_sim::object::Capsule *capsule = new dart_sim::object::Capsule(radius, height, mass);
  dynamicsWorld_->addSkeleton(capsule->getSkeletonPtr());
  objectList_.push_back(capsule);
  return capsule;
}

object::CheckerBoard *World::addCheckerboard(double gridSize,
                                             double xLength,
                                             double yLength,
                                             double reflectanceI,
                                             benchmark::CollisionGroupType collisionGroup,
                                             benchmark::CollisionGroupType collisionMask) {
  dart_sim::object::CheckerBoard *checkerBoard = new dart_sim::object::CheckerBoard(xLength, yLength);
  dynamicsWorld_->addSkeleton(checkerBoard->getSkeletonPtr());
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

void World::setGravity(const Eigen::Vector3d &gravity) {
  gravity_ = gravity;
  dynamicsWorld_->setGravity(gravity);
}

void World::integrate() {
  dynamicsWorld_->step();
}

void World::setTimeStep(double timeStep) {
  World::timeStep_ = timeStep;
  dynamicsWorld_->setTimeStep(timeStep);
}

} // dart_sim
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
  auto *sphere = new dart_sim::object::Sphere(radius, mass, objectList_.size());
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
  auto *box = new dart_sim::object::Box(xLength, yLength, zLength, mass, objectList_.size());
  dynamicsWorld_->addSkeleton(box->getSkeletonPtr());
  objectList_.push_back(box);
  return box;
}

object::Capsule *World::addCapsule(double radius,
                                   double height,
                                   double mass,
                                   benchmark::CollisionGroupType collisionGroup,
                                   benchmark::CollisionGroupType collisionMask) {
  auto *capsule = new dart_sim::object::Capsule(radius, height, mass, 0);
  dynamicsWorld_->addSkeleton(capsule->getSkeletonPtr());
  objectList_.push_back(capsule);
  return capsule;
}

object::CheckerBoard *World::addCheckerboard(double gridSize,
                                             double xLength,
                                             double yLength,
                                             double reflectanceI,
                                             bo::CheckerboardShape shape,
                                             benchmark::CollisionGroupType collisionGroup,
                                             benchmark::CollisionGroupType collisionMask) {
  RAIFATAL_IF(shape == bo::PLANE_SHAPE, "plane shape ground is not supported")

  auto *checkerBoard = new dart_sim::object::CheckerBoard(xLength, yLength, shape, objectList_.size());
  dynamicsWorld_->addSkeleton(checkerBoard->getSkeletonPtr());
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

object::Cylinder *World::addCylinder(double radius,
                                     double height,
                                     double mass,
                                     benchmark::CollisionGroupType collisionGroup,
                                     benchmark::CollisionGroupType collisionMask) {
  auto *cylinder = new dart_sim::object::Cylinder(radius, height, mass, 0);
  dynamicsWorld_->addSkeleton(cylinder->getSkeletonPtr());
  objectList_.push_back(cylinder);
  return cylinder;
}

object::ArticulatedSystem *World::addArticulatedSystem(std::string urdfPath,
                                                       benchmark::CollisionGroupType collisionGroup,
                                                       benchmark::CollisionGroupType collisionMask) {
  auto *robot = new dart_sim::object::ArticulatedSystem(urdfPath);
  dynamicsWorld_->addSkeleton(robot->getSkeletonPtr());
  objectList_.push_back(robot);
  return robot;
}

void World::setGravity(const benchmark::Vec<3> &gravity) {
  gravity_[0] = gravity[0];
  gravity_[1] = gravity[1];
  gravity_[2] = gravity[2];
  dynamicsWorld_->setGravity(gravity_);
}

void World::integrate() {
  dynamicsWorld_->step();
}

void World::setTimeStep(double timeStep) {
  timeStep_ = timeStep;
  dynamicsWorld_->setTimeStep(timeStep);
}

void World::integrate(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate() instead")
}

int World::getNumObject() {
  return objectList_.size();
}

} // dart_sim
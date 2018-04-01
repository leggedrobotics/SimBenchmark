//
// Created by kangd on 16.03.18.
//

#include "World_RG.hpp"

namespace dart_sim {

World_RG::World_RG(int windowWidth, int windowHeight, float cms, int flags) :
    world_(),
    benchmark::World_RG(windowWidth, windowHeight, cms, flags) {}

World_RG::World_RG() :
    world_(),
    benchmark::World_RG() {}

benchmark::SingleBodyHandle World_RG::addSphere(double radius,
                                                double mass,
                                                benchmark::CollisionGroupType collisionGroup,
                                                benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addSphere(radius, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle World_RG::addBox(double xLength,
                                             double yLength,
                                             double zLength,
                                             double mass,
                                             benchmark::CollisionGroupType collisionGroup,
                                             benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addBox(xLength, yLength, zLength, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Box(xLength, yLength, zLength, true));
  processSingleBody(handle);
  return handle;
}

World_RG::~World_RG() {
  if(!isEnded_ && isReady_)
    visEnd();
}

benchmark::SingleBodyHandle World_RG::addCapsule(double radius,
                                                 double height,
                                                 double mass,
                                                 benchmark::CollisionGroupType collisionGroup,
                                                 benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addCapsule(radius, height, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle World_RG::addCheckerboard(double gridSize,
                                                      double xLength,
                                                      double yLength,
                                                      double reflectanceI,
                                                      benchmark::CollisionGroupType collisionGroup,
                                                      benchmark::CollisionGroupType collisionMask,
                                                      int flags) {
  benchmark::SingleBodyHandle handle(world_.addCheckerboard(gridSize, xLength, yLength, reflectanceI, collisionGroup, collisionMask), {}, {});
  handle.hidable = false;
  if(gui_) {
    handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
    static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & benchmark::GRID;
    gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
  }
  sbHandles_.push_back(handle);
  return handle;
}

void World_RG::setGravity(Eigen::Vector3d gravity) {
  world_.setGravity(gravity);
}

void World_RG::setTimeStep(double timeStep) {
  timeStep_ = timeStep;
  world_.setTimeStep(timeStep);
}

void World_RG::integrate() {
  world_.integrate();
}

void World_RG::loop(double realTimeFactor) {
  while (visualizerLoop(timeStep_, realTimeFactor))
    integrate();
}

void World_RG::loop(double dt, double realTimeFactor) {
}

void World_RG::integrate(double dt) {
}

void World_RG::setERP(double erp, double erp2, double frictionErp) {

}

} // dart_sim
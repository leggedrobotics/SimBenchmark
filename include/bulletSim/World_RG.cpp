//
// Created by kangd on 10.02.18.
//

#include "World_RG.hpp"

namespace bullet_sim {

World_RG::World_RG(int windowWidth, int windowHeight, float cms, int flags, SolverOption solverOption) :
    solverOption_(solverOption),
    world_(solverOption),
    benchmark::World_RG(windowWidth, windowHeight, cms, flags) {}

World_RG::World_RG(SolverOption solverOption) :
    world_(solverOption),
    benchmark::World_RG() {}

World_RG::~World_RG() {
  if(!isEnded_ && isReady_)
    visEnd();
}

benchmark::SingleBodyHandle World_RG::addBox(double xLength,
                                  double yLength,
                                  double zLength,
                                  double mass,
                                  CollisionGroupType collisionGroup,
                                  CollisionGroupType collisionMask) {

  benchmark::SingleBodyHandle handle(world_.addBox(xLength, yLength, zLength, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Box(xLength, yLength, zLength, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle World_RG::addSphere(double radius,
                                     double mass,
                                     CollisionGroupType collisionGroup,
                                     CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addSphere(radius, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle World_RG::addCheckerboard(double gridSize,
                                           double xLength,
                                           double yLength,
                                           double reflectanceI,
                                           CollisionGroupType collisionGroup,
                                           CollisionGroupType collisionMask,
                                           int flags) {
  benchmark::SingleBodyHandle handle(world_.addCheckerboard(gridSize, xLength, yLength, reflectanceI, collisionGroup, collisionMask), {}, {});
  handle.hidable = false;
  if(gui_) {
    handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
    static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & GRID;
    gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
  }
  sbHandles_.push_back(handle);
  return handle;
}

benchmark::SingleBodyHandle World_RG::addCapsule(double radius,
                                      double height,
                                      double mass,
                                      CollisionGroupType collisionGroup,
                                      CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addCapsule(radius, height, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
  processSingleBody(handle);
  return handle;
}

void World_RG::integrate(double dt) {
  world_.integrate(dt);
}

void World_RG::setGravity(Eigen::Vector3d gravity) {
  world_.setGravity({gravity.x(), gravity.y(), gravity.z()});
}

void World_RG::setERP(double erp, double erp2, double frictionErp) {
  world_.setERP(erp, erp2, frictionErp);
}

} // bullet_sim
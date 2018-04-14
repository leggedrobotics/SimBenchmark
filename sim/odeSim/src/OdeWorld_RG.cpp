//
// Created by kangd on 10.02.18.
//

#include "OdeWorld_RG.hpp"

namespace ode_sim {

OdeWorld_RG::OdeWorld_RG(int windowWidth, int windowHeight, float cms, int flags, SolverOption solverOption) :
    world_(solverOption),
    benchmark::World_RG(windowWidth, windowHeight, cms, flags) {}

OdeWorld_RG::OdeWorld_RG(SolverOption solverOption) :
    world_(solverOption),
    benchmark::World_RG() {}

OdeWorld_RG::~OdeWorld_RG() {
  if(!isEnded_ && isReady_)
    visEnd();
}

benchmark::SingleBodyHandle OdeWorld_RG::addBox(double xLength,
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

benchmark::SingleBodyHandle OdeWorld_RG::addSphere(double radius,
                                                   double mass,
                                                   benchmark::CollisionGroupType collisionGroup,
                                                   benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addSphere(radius, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle OdeWorld_RG::addCapsule(double radius,
                                                    double height,
                                                    double mass,
                                                    benchmark::CollisionGroupType collisionGroup,
                                                    benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addCapsule(radius, height, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle OdeWorld_RG::addCylinder(double radius,
                                          double height,
                                          double mass,
                                          benchmark::CollisionGroupType collisionGroup,
                                          benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addCylinder(radius, height, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Cylinder(radius, height, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle OdeWorld_RG::addCheckerboard(double gridSize,
                                                         double xLength,
                                                         double yLength,
                                                         double reflectanceI,
                                                         bo::CheckerboardShape shape,
                                                         benchmark::CollisionGroupType collisionGroup,
                                                         benchmark::CollisionGroupType collisionMask,
                                                         int flags) {
  benchmark::SingleBodyHandle handle(world_.addCheckerboard(gridSize,
                                                            xLength,
                                                            yLength,
                                                            reflectanceI,
                                                            bo::PLANE_SHAPE,
                                                            collisionGroup,
                                                            collisionMask), {}, {});
  handle.hidable = false;
  if(gui_) {
    handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
    static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & bo::GRID;
    gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
  }
  sbHandles_.push_back(handle);
  return handle;
}

void OdeWorld_RG::integrate(double dt) {
  world_.integrate(dt);
}

void OdeWorld_RG::setGravity(Eigen::Vector3d gravity) {
  world_.setGravity({gravity.x(), gravity.y(), gravity.z()});
}
void OdeWorld_RG::setERP(double erp, double erp2, double frictionErp) {
  world_.setERP(erp);
}

int OdeWorld_RG::getNumObject() {
  return world_.getNumObject();
}

} // ode_sim
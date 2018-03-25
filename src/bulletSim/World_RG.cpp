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
                                             benchmark::CollisionGroupType collisionGroup,
                                             benchmark::CollisionGroupType collisionMask) {

  benchmark::SingleBodyHandle handle(
      world_.addBox(xLength, yLength, zLength, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Box(xLength, yLength, zLength, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle World_RG::addSphere(double radius,
                                                double mass,
                                                benchmark::CollisionGroupType collisionGroup,
                                                benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(
      world_.addSphere(radius, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
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
  benchmark::SingleBodyHandle handle(
      world_.addCheckerboard(gridSize, xLength, yLength, reflectanceI, collisionGroup, collisionMask), {}, {});
  handle.hidable = false;
  if(gui_) {
    handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
    static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & benchmark::GRID;
    gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
  }
  sbHandles_.push_back(handle);
  return handle;
}

benchmark::SingleBodyHandle World_RG::addCapsule(double radius,
                                                 double height,
                                                 double mass,
                                                 benchmark::CollisionGroupType collisionGroup,
                                                 benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(
      world_.addCapsule(radius, height, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
  processSingleBody(handle);
  return handle;
}

benchmark::ArticulatedSystemHandle World_RG::addArticulatedSystem(std::string nm,
                                                                  CollisionGroupType collisionGroup,
                                                                  CollisionGroupType collisionMask) {
  benchmark::ArticulatedSystemHandle handle(
      world_.addArticulatedSystem(nm, collisionGroup, collisionMask), {}, {});
  if(!gui_) {
    asHandles_.push_back(handle);
    return handle;
  }

//  for (int i = 0; i < handle->visObj.size(); i++) {
//    switch (std::get<3>(handle->visObj[i])) {
//      case benchmark::object::Shape::Box:
//        handle.visual().push_back(new rai_graphics::object::Box(handle->visProps_[i].second.v[0],
//                                                                handle->visProps_[i].second.v[1],
//                                                                handle->visProps_[i].second.v[2], true));
//        break;
//      case benchmark::object::Shape::Cylinder:
//        handle.visual().push_back(new rai_graphics::object::Cylinder(handle->visProps_[i].second.v[0],
//                                                                     handle->visProps_[i].second.v[1], true));
//        break;
//      case benchmark::object::Shape::Sphere:
//        handle.visual().push_back(new rai_graphics::object::Sphere(handle->visProps_[i].second.v[0], true));
//        break;
//      case benchmark::object::Shape::Mesh:
//        checkFileExistance(nm + handle->visProps_[i].first);
//        handle.visual().push_back(new rai_graphics::object::Mesh(nm + handle->visProps_[i].first,
//                                                                 handle->visProps_[i].second.v[0]));
//        break;
//    }
//    handle.visual().back()->setColor({float(std::get<4>(handle->visObj[i]).v[0]),
//                                      float(std::get<4>(handle->visObj[i]).v[1]),
//                                      float(std::get<4>(handle->visObj[i]).v[2])});
//    processGraphicalObject(handle.visual().back(), std::get<2>(handle->visObj[i]));
//  }
//
//  for (int i = 0; i < handle->visColObj.size(); i++) {
//    switch (std::get<3>(handle->visColObj[i])) {
//      case benchmark::object::Shape::Box:
//        handle.alternateVisual().push_back(new rai_graphics::object::Box(handle->visColProps_[i].second.v[0],
//                                                                         handle->visColProps_[i].second.v[1],
//                                                                         handle->visColProps_[i].second.v[2],
//                                                                         true));
//        break;
//      case benchmark::object::Shape::Cylinder:
//        handle.alternateVisual().push_back(new rai_graphics::object::Cylinder(handle->visColProps_[i].second.v[0],
//                                                                              handle->visColProps_[i].second.v[1],
//                                                                              true));
//        break;
//      case benchmark::object::Shape::Sphere:
//        handle.alternateVisual().push_back(new rai_graphics::object::Sphere(handle->visColProps_[i].second.v[0],
//                                                                            true));
//        break;
//      case benchmark::object::Shape::Mesh:
//      RAIFATAL("mesh collision body is not supported yet");
//        break;
//      default: RAIFATAL("unsupported type: ");
//        break;
//    }
//    processGraphicalObject(handle.alternateVisual().back(), std::get<2>(handle->visColObj[i]));
//  }

  asHandles_.push_back(handle);
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
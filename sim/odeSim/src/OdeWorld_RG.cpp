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

void OdeWorld_RG::updateFrame() {
  RAIFATAL_IF(!gui_, "use different constructor for visualization")
  const bool showAlternateGraphicsIfexists = gui_->getCustomToggleState(3);

//  TODO articulated system
  for (auto &as : asHandles_) {
    benchmark::Vec<4> quat;
    benchmark::Vec<3> pos;
    benchmark::Vec<4> color;

    // update visuals for articulated system
//    as->updateVisuals();

    if (showAlternateGraphicsIfexists) {
      /// update collision objects
      for (int i = 0; i < as->getVisColOb().size(); i++) {
        as.alternateVisual()[i]->setVisibility(true);
        pos = std::get<1>(as->getVisColOb()[i]);
        as.alternateVisual()[i]->setPos(
            pos[0],
            pos[1],
            pos[2]);
        rotMatToQuat(std::get<0>(as->getVisColOb()[i]), quat);
        as.alternateVisual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        adjustTransparency(as.alternateVisual()[i], as.hidable);
      }

      for (int i = 0; i < as->getVisOb().size(); i++)
        as.visual()[i]->setVisibility(false);

    } else {
      for (int i = 0; i < as->getVisOb().size(); i++) {
        as.visual()[i]->setVisibility(true);
        if (!as.visual()[i]->isVisible()) continue;
        pos = std::get<1>(as->getVisOb()[i]);
        color = std::get<4>(as->getVisOb()[i]);
        as.visual()[i]->setPos(
            pos[0],
            pos[1],
            pos[2]
        );
        rotMatToQuat(std::get<0>(as->getVisOb()[i]), quat);
        as.visual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        as.visual()[i]->setColor({float(color[0]),
                                  float(color[1]),
                                  float(color[2])});
        as.visual()[i]->setTransparency(float(color[3]));
        adjustTransparency(as.visual()[i], as.hidable);
      }
      for (int i = 0; i < as->getVisColOb().size(); i++)
        as.alternateVisual()[i]->setVisibility(false);
    }
  }
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

ArticulatedSystemHandle OdeWorld_RG::addArticulatedSystem(std::string nm,
                                                          benchmark::CollisionGroupType collisionGroup,
                                                          benchmark::CollisionGroupType collisionMask) {
  ArticulatedSystemHandle handle(
      world_.addArticulatedSystem(nm, collisionGroup, collisionMask), {}, {});
  if(!gui_) {
    asHandles_.push_back(handle);
    return handle;
  }

  for (int i = 0; i < handle->visObj.size(); i++) {
    switch (std::get<3>(handle->visObj[i])) {
      case benchmark::object::Shape::Box:
        handle.visual().push_back(new rai_graphics::object::Box(handle->visProps_[i].second.v[0],
                                                                handle->visProps_[i].second.v[1],
                                                                handle->visProps_[i].second.v[2], true));
        break;
      case benchmark::object::Shape::Cylinder:
        handle.visual().push_back(new rai_graphics::object::Cylinder(handle->visProps_[i].second.v[0],
                                                                     handle->visProps_[i].second.v[1], true));
        break;
      case benchmark::object::Shape::Sphere:
        handle.visual().push_back(new rai_graphics::object::Sphere(handle->visProps_[i].second.v[0], true));
        break;
      case benchmark::object::Shape::Mesh:
        checkFileExistance(nm + handle->visProps_[i].first);
        handle.visual().push_back(new rai_graphics::object::Mesh(nm + handle->visProps_[i].first,
                                                                 handle->visProps_[i].second.v[0]));
        break;
    }
    handle.visual().back()->setColor({float(std::get<4>(handle->visObj[i]).v[0]),
                                      float(std::get<4>(handle->visObj[i]).v[1]),
                                      float(std::get<4>(handle->visObj[i]).v[2])});
    processGraphicalObject(handle.visual().back(), std::get<2>(handle->visObj[i]));
  }

  for (int i = 0; i < handle->visColObj.size(); i++) {
    switch (std::get<3>(handle->visColObj[i])) {
      case benchmark::object::Shape::Box:
        handle.alternateVisual().push_back(new rai_graphics::object::Box(handle->visColProps_[i].second.v[0],
                                                                         handle->visColProps_[i].second.v[1],
                                                                         handle->visColProps_[i].second.v[2],
                                                                         true));
        break;
      case benchmark::object::Shape::Cylinder:
        handle.alternateVisual().push_back(new rai_graphics::object::Cylinder(handle->visColProps_[i].second.v[0],
                                                                              handle->visColProps_[i].second.v[1],
                                                                              true));
        break;
      case benchmark::object::Shape::Sphere:
        handle.alternateVisual().push_back(new rai_graphics::object::Sphere(handle->visColProps_[i].second.v[0],
                                                                            true));
        break;
      case benchmark::object::Shape::Mesh:
      RAIFATAL("mesh collision body is not supported yet");
        break;
      default: RAIFATAL("unsupported type: ");
        break;
    }
    processGraphicalObject(handle.alternateVisual().back(), std::get<2>(handle->visColObj[i]));
  }

  asHandles_.push_back(handle);
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
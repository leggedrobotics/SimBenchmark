//
// Created by kangd on 18.02.18.
//

#include "World_RG.hpp"

mujoco_sim::World_RG::World_RG(int windowWidth,
                               int windowHeight,
                               float cms,
                               const char *modelPath,
                               int flags,
                               mujoco_sim::SolverOption solverOption) :
    world_(modelPath),
    benchmark::World_RG(windowWidth, windowHeight, cms, flags) {
  initFromModel();
}

mujoco_sim::World_RG::World_RG(const char *modelPath, mujoco_sim::SolverOption solverOption) :
    world_(modelPath),
    benchmark::World_RG() {
  initFromModel();
}

void mujoco_sim::World_RG::initFromModel() {

  mjModel *model = world_.getWorldModel();

  // make objects
  for(int i = 0; i < model->ngeom; i++) {
    mjtNum *geomSize = model->geom_size + i * 3;
    RAIINFO(geomSize[0] << geomSize[1] << geomSize[2]);

    switch (*(model->geom_type + i)) {

      case mjGEOM_PLANE:
        // geomsize = (xlength, ylength, gridsize)
        addCheckerboard(geomSize[2], geomSize[0], geomSize[1], 0.1);
        break;
      case mjGEOM_SPHERE:
        // geomsize = radius
        addSphere(geomSize[0], 1);
        break;
      case mjGEOM_CAPSULE:
        // geomsize = (radius, height * 0.5)
        addCapsule(geomSize[0], geomSize[1] * 2, 1);
      case mjGEOM_BOX:
        // geomsize = (xlength, ylength, zlength)
        addBox(geomSize[0], geomSize[1], geomSize[2], 1);
        break;
      default:
      RAIFATAL("wrong geometry type");
    }

    objectIndex_++;
  }
}

mujoco_sim::World_RG::~World_RG() {

}

benchmark::SingleBodyHandle mujoco_sim::World_RG::addSphere(double radius,
                                                            double mass,
                                                            CollisionGroupType collisionGroup,
                                                            CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addSphere(radius, mass, objectIndex_, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::World_RG::addBox(double xLength,
                                                         double yLength,
                                                         double zLength,
                                                         double mass,
                                                         CollisionGroupType collisionGroup,
                                                         CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addBox(xLength, yLength, zLength, mass, objectIndex_, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Box(xLength, yLength, zLength, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::World_RG::addCheckerboard(double gridSize,
                                                                  double xLength,
                                                                  double yLength,
                                                                  double reflectanceI,
                                                                  CollisionGroupType collisionGroup,
                                                                  CollisionGroupType collisionMask,
                                                                  int flags) {
  benchmark::SingleBodyHandle handle(world_.addCheckerboard(gridSize, xLength, yLength, reflectanceI, objectIndex_, collisionGroup, collisionMask), {}, {});
  handle.hidable = false;
  if(gui_) {
    handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
    static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & benchmark::GRID;
    gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
  }
  sbHandles_.push_back(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::World_RG::addCapsule(double radius,
                                                             double height,
                                                             double mass,
                                                             CollisionGroupType collisionGroup,
                                                             CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addCapsule(radius, height, mass, objectIndex_, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
  processSingleBody(handle);
  return handle;
}

void mujoco_sim::World_RG::integrate(double dt) {
  world_.integrate(dt);
}
void mujoco_sim::World_RG::setGravity(Eigen::Vector3d gravity) {

}
void mujoco_sim::World_RG::setERP(double erp, double erp2, double frictionErp) {

}
const std::vector<mujoco_sim::object::SingleBodyObject *> &mujoco_sim::World_RG::getObjectList() const {
  return world_.getObjectList();
}

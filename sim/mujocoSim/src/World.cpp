//
// Created by kangd on 18.02.18.
//

#include "World.hpp"

namespace mujoco_sim {

mujoco_sim::World::World(const char *modelPath,
                         const char *keyPath,
                         SolverOption solverOption) {

  // activate MuJoCo Pro
  mj_activate(keyPath);

  // load model
  char error[1000];

  worldModel_ = mj_loadXML(modelPath, NULL, error, 1000);
  if( !worldModel_ )
  {
    RAIFATAL(error);
  }

  // set solver
  switch (solverOption) {
    case SOLVER_PGS:
      worldModel_->opt.solver = mjSOL_PGS;
      break;
    case SOLVER_NEWTON:
      worldModel_->opt.solver = mjSOL_NEWTON;
      break;
    case SOLVER_CG:
      worldModel_->opt.solver = mjSOL_CG;
      break;
    default:
      worldModel_->opt.solver = mjSOL_PGS;
  }

  // make data corresponding to model
  worldData_ = mj_makeData(worldModel_);
}

mujoco_sim::World::~World() {

  // remove objects
  for (auto *ob: objectList_)
    delete ob;

  // free model and data, deactivate
  mj_deleteData(worldData_);
  mj_deleteModel(worldModel_);
  mj_deactivate();

}

mjModel *mujoco_sim::World::getWorldModel() const {
  return worldModel_;
}
mjData *mujoco_sim::World::getWorldData() const {
  return worldData_;
}

void mujoco_sim::World::integrate(double dt) {
  if(worldModel_->opt.timestep != dt)
    worldModel_->opt.timestep = dt;

  mj_step(worldModel_, worldData_);
}

object::Sphere *World::addSphere(double radius,
                                 double mass,
                                 int bodyId,
                                 int geomId) {
  object::Sphere *sphere = new object::Sphere(radius, mass, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(sphere);
  return sphere;
}

object::Box *World::addBox(double xLength,
                           double yLength,
                           double zLength,
                           double mass,
                           int bodyId,
                           int geomId) {
  object::Box *box = new object::Box(xLength, yLength, zLength, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(box);
  return box;
}

object::CheckerBoard *World::addCheckerboard(double gridSize,
                                             double xLength,
                                             double yLength,
                                             double reflectanceI,
                                             int bodyId,
                                             int geomId) {
  object::CheckerBoard *checkerBoard =
      new object::CheckerBoard(xLength, yLength, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

object::Capsule *World::addCapsule(double radius,
                                   double height,
                                   double mass,
                                   int bodyId,
                                   int geomId) {
  object::Capsule *capsule = new object::Capsule(radius, height, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(capsule);
  return capsule;
}

object::Cylinder *World::addCylinder(double radius,
                                     double height,
                                     double mass,
                                     int bodyId,
                                     int geomId) {
  object::Cylinder *cylinder = new object::Cylinder(radius, height, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(cylinder);
  return cylinder;
}

} // mujoco_sim

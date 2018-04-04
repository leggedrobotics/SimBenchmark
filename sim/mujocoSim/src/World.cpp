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
  simOption_ = &worldModel_->opt;

  // init simulation option
  simOption_->gravity[0] = 0;
  simOption_->gravity[1] = 0;
  simOption_->gravity[2] = -9.81;

  // init variables
  dof_ = worldModel_->nv;
  numActuators_ = worldModel_->na;
  dimGenCoord_ = worldModel_->nq;

  generalizedCoordinate_.resize(dimGenCoord_);
  generalizedCoordinate_.setZero();
  generalizedVelocity_.resize(dof_);
  generalizedVelocity_.setZero();
  generalizedForce_.resize(dof_);
  generalizedForce_.setZero();
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

int World::getDOF() {
  return dof_;
}

int World::getGeneralizedCoordinateDim() {
  return dimGenCoord_;
}

const EigenVec World::getGeneralizedCoordinate() {
  for(int i = 0; i < dimGenCoord_; i++)
    generalizedCoordinate_[i] = worldData_->qpos[i];
  return generalizedCoordinate_.e();
}

const EigenVec World::getGeneralizedVelocity() {
  for(int i = 0; i < dof_; i++)
    generalizedVelocity_[i] = worldData_->qvel[i];
  return generalizedVelocity_.e();
}

const EigenVec World::getGeneralizedForce() {
  for(int i = 0; i < dof_; i++) {
    generalizedForce_[i] = worldData_->qfrc_applied[i];
  }
  return generalizedForce_.e();
}

void World::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  RAIFATAL_IF(jointState.size() != dimGenCoord_, "invalid generalized coordinate input")
  for(int i = 0; i < dimGenCoord_; i++) {
    generalizedCoordinate_[i] = jointState[i];
    worldData_->qpos[i] = jointState[i];
  }
}

void World::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  RAIFATAL_IF(jointState.size() != dimGenCoord_, "invalid generalized coordinate input")
  for(int i = 0; i < dimGenCoord_; i++) {
    generalizedCoordinate_[i] = jointState.begin()[i];
    worldData_->qpos[i] = jointState.begin()[i];
  }
}

void World::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = jointVel[i];
    worldData_->qvel[i] = jointVel[i];
  }
}

void World::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = jointVel.begin()[i];
    worldData_->qvel[i] = jointVel.begin()[i];
  }
}

void World::setGeneralizedForce(std::initializer_list<double> tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  for(int i = 0; i < dof_; i++) {
    generalizedForce_[i] = tau.begin()[i];
    worldData_->qfrc_applied[i] = tau.begin()[i];
  }
}

void World::setGeneralizedForce(const Eigen::VectorXd &tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  for(int i = 0; i < dof_; i++) {
    generalizedForce_[i] = tau[i];
    worldData_->qfrc_applied[i] = tau[i];
  }
}
void World::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
  RAIFATAL_IF(genco.size() != dimGenCoord_, "invalid generalized coordinate input")
  RAIFATAL_IF(genvel.size() != dof_, "invalid generalized velocity input")

  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = worldData_->qvel[i];
    genvel[i] = generalizedVelocity_[i];
  }

  for(int i = 0; i < dof_; i++) {
    generalizedCoordinate_[i] = worldData_->qpos[i];
    genco[i] = generalizedCoordinate_[i];
  }
}

void World::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
  RAIFATAL_IF(genco.size() != dimGenCoord_, "invalid generalized coordinate input")
  RAIFATAL_IF(genvel.size() != dof_, "invalid generalized velocity input")

  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = genvel[i];
    worldData_->qvel[i] = generalizedVelocity_[i];
  }

  for(int i = 0; i < dof_; i++) {
    generalizedCoordinate_[i] = genco[i];
    worldData_->qpos[i] = generalizedCoordinate_[i];
  }
}

void World::setGravity(const benchmark::Vec<3> &gravity) {
  simOption_->gravity[0] = gravity[0];
  simOption_->gravity[1] = gravity[1];
  simOption_->gravity[2] = gravity[2];
}

int World::getWorldNumContacts() {
  return worldData_->ncon;
}

} // mujoco_sim

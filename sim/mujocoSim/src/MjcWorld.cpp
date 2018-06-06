//
// Created by kangd on 18.02.18.
//

#include "MjcWorld.hpp"

namespace mujoco_sim {

mujoco_sim::MjcWorld::MjcWorld(const char *modelPath,
                               const char *keyPath,
                               SolverOption solver,
                               IntegratorOption integrator) {

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
  switch (solver) {
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

  // set integrator
  switch (integrator) {
    case INTEGRATOR_EULER:
      worldModel_->opt.integrator = mjINT_EULER;
      break;
    case INTEGRATOR_RK4:
      worldModel_->opt.integrator = mjINT_RK4;
      break;
    default:
      worldModel_->opt.integrator = mjINT_EULER;
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

  // energy calculation
  simOption_->enableflags |= mjENBL_ENERGY;
}

mujoco_sim::MjcWorld::~MjcWorld() {

  // remove objects
  for (auto *ob: objectList_)
    delete ob;

  // free model and data, deactivate
  mj_deleteData(worldData_);
  mj_deleteModel(worldModel_);
  mj_deactivate();

}

mjModel *mujoco_sim::MjcWorld::getWorldModel() const {
  return worldModel_;
}
mjData *mujoco_sim::MjcWorld::getWorldData() const {
  return worldData_;
}


object::MjcSphere *MjcWorld::addSphere(double radius,
                                 double mass,
                                 int bodyId,
                                 int geomId) {
  object::MjcSphere *sphere = new object::MjcSphere(radius, mass, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(sphere);
  return sphere;
}

object::MjcBox *MjcWorld::addBox(double xLength,
                           double yLength,
                           double zLength,
                           double mass,
                           int bodyId,
                           int geomId) {
  object::MjcBox *box = new object::MjcBox(xLength, yLength, zLength, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(box);
  return box;
}

object::MjcCheckerBoard *MjcWorld::addCheckerboard(double gridSize,
                                             double xLength,
                                             double yLength,
                                             double reflectanceI,
                                             bo::CheckerboardShape shape,
                                             int bodyId,
                                             int geomId) {
  RAIFATAL_IF(shape == bo::BOX_SHAPE, "box shape ground is not supported")

  object::MjcCheckerBoard *checkerBoard =
      new object::MjcCheckerBoard(xLength, yLength, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

object::MjcCapsule *MjcWorld::addCapsule(double radius,
                                   double height,
                                   double mass,
                                   int bodyId,
                                   int geomId) {
  object::MjcCapsule *capsule = new object::MjcCapsule(radius, height, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(capsule);
  return capsule;
}

object::MjcCylinder *MjcWorld::addCylinder(double radius,
                                     double height,
                                     double mass,
                                     int bodyId,
                                     int geomId) {
  object::MjcCylinder *cylinder = new object::MjcCylinder(radius, height, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(cylinder);
  return cylinder;
}

int MjcWorld::getDOF() {
  return dof_;
}

int MjcWorld::getGeneralizedCoordinateDim() {
  return dimGenCoord_;
}

const EigenVec MjcWorld::getGeneralizedCoordinate() {
  for(int i = 0; i < dimGenCoord_; i++)
    generalizedCoordinate_[i] = worldData_->qpos[i];
  return generalizedCoordinate_.e();
}

const EigenVec MjcWorld::getGeneralizedVelocity() {
  for(int i = 0; i < dof_; i++)
    generalizedVelocity_[i] = worldData_->qvel[i];
  return generalizedVelocity_.e();
}

const EigenVec MjcWorld::getGeneralizedForce() {
  for(int i = 0; i < dof_; i++) {
    generalizedForce_[i] = worldData_->qfrc_applied[i];
  }
  return generalizedForce_.e();
}

void MjcWorld::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  RAIFATAL_IF(jointState.size() != dimGenCoord_, "invalid generalized coordinate input")
  for(int i = 0; i < dimGenCoord_; i++) {
    generalizedCoordinate_[i] = jointState[i];
    worldData_->qpos[i] = jointState[i];
  }
}

void MjcWorld::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  RAIFATAL_IF(jointState.size() != dimGenCoord_, "invalid generalized coordinate input")
  for(int i = 0; i < dimGenCoord_; i++) {
    generalizedCoordinate_[i] = jointState.begin()[i];
    worldData_->qpos[i] = jointState.begin()[i];
  }
}

void MjcWorld::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = jointVel[i];
    worldData_->qvel[i] = jointVel[i];
  }
}

void MjcWorld::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = jointVel.begin()[i];
    worldData_->qvel[i] = jointVel.begin()[i];
  }
}

void MjcWorld::setGeneralizedForce(std::initializer_list<double> tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  for(int i = 0; i < dof_; i++) {
    generalizedForce_[i] = tau.begin()[i];
    worldData_->qfrc_applied[i] = tau.begin()[i];
  }
}

void MjcWorld::setGeneralizedForce(const Eigen::VectorXd &tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  for(int i = 0; i < dof_; i++) {
    generalizedForce_[i] = tau[i];
    worldData_->qfrc_applied[i] = tau[i];
  }
}
void MjcWorld::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
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

void MjcWorld::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
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

void MjcWorld::setGravity(const benchmark::Vec<3> &gravity) {
  simOption_->gravity[0] = gravity[0];
  simOption_->gravity[1] = gravity[1];
  simOption_->gravity[2] = gravity[2];
}

int MjcWorld::getWorldNumContacts() {
  return worldData_->ncon;
}

int MjcWorld::getNumObject() {
  return objectList_.size();
}

void MjcWorld::setNoSlipParameter(int maxIter) {
  simOption_->noslip_iterations = maxIter;
}

void MjcWorld::setTimeStep(double timeStep) {
  worldModel_->opt.timestep = timeStep;
}

void MjcWorld::integrate() {
  mj_step(worldModel_, worldData_);
}

void MjcWorld::integrate1() {
  mj_step1(worldModel_, worldData_);
}

void MjcWorld::integrate2() {
  mj_step2(worldModel_, worldData_);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1>> MjcWorld::getLinearMomentumInCartesianSpace() {
  Eigen::Vector3d linearMomentum;
  linearMomentum.setZero();
  for(int i = 0; i < objectList_.size(); i++) {
    if(!static_cast<object::MjcSingleBodyObject *>(objectList_[i])->isMovable()) continue;

    linearMomentum += objectList_[i]->getMass() * objectList_[i] -> getLinearVelocity();
  }
  linearMomentum_ = {linearMomentum.x(), linearMomentum.y(), linearMomentum.z()};
  return linearMomentum_.e();
}

double MjcWorld::getTotalMass() {
  double mass = 0;
  for(int i = 0; i < objectList_.size(); i++) {
    if(!static_cast<object::MjcSingleBodyObject *>(objectList_[i])->isMovable()) continue;
    mass += objectList_[i]->getMass();
  }
  return mass;
}

void mujoco_sim::MjcWorld::integrate(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate() instead")
}
void MjcWorld::integrate1(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate1() instead")
}
void MjcWorld::integrate2(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate2() instead")
}
void MjcWorld::forwardKinematics() {
  mj_forward(worldModel_, worldData_);
}

void MjcWorld::resetSimulation() {
  mj_resetData(worldModel_, worldData_);
  mj_forward(worldModel_, worldData_);
}

} // mujoco_sim

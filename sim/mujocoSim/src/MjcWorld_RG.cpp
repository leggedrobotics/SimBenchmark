//
// Created by kangd on 18.02.18.
//

#include "MjcWorld_RG.hpp"

// the mass data is from XML
#define MJC_MASS_FROM_XML -1

mujoco_sim::MjcWorld_RG::MjcWorld_RG(int windowWidth,
                                     int windowHeight,
                                     float cms,
                                     const char *modelPath,
                                     const char *keyPath,
                                     int flags,
                                     SolverOption solver,
                                     IntegratorOption integrator) :
    world_(modelPath, keyPath, solver, INTEGRATOR_RK4),
    benchmark::World_RG(windowWidth, windowHeight, cms, flags) {
  initFromModel();
}

mujoco_sim::MjcWorld_RG::MjcWorld_RG(const char *modelPath,
                                     const char *keyPath,
                                     SolverOption solver,
                                     IntegratorOption integrator) :
    world_(modelPath, keyPath, solver, INTEGRATOR_RK4),
    benchmark::World_RG() {
  initFromModel();
}

void mujoco_sim::MjcWorld_RG::initFromModel() {

  mjModel *model = world_.getWorldModel();

  // make objects
  for(int i = 0; i < model->nbody; i++) {
    const int geomNumInBody = model->body_geomnum[i];
    const int geomAddrInBody = model->body_geomadr[i];

    std::stringstream bodyname;
    for(int j = model->name_bodyadr[i]; i+1 <= model->nbody && j < model->name_bodyadr[i+1] - 1; j++) {
      bodyname << model->names[j];
    }

    if(bodyname.str() == "ground") {
      // checkerboard if the name of the body is ground
      addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, i, 0, bo::GRID);
      return;
    }

    for(int j = 0; j < geomNumInBody; j++) {
      const int geomIndex = geomAddrInBody + j;

      mjtNum *geomSize = model->geom_size + geomIndex * 3;
      int geomType = model->geom_type[geomIndex];

      switch (geomType) {
        case mjGEOM_PLANE: {
          // geomsize = (xlength, ylength, gridsize)
          addCheckerboard(geomSize[2], geomSize[0], geomSize[1], 0.1, bo::PLANE_SHAPE, i, j, bo::GRID);
          break;
        }
        case mjGEOM_SPHERE: {
          // geomsize = radius_
          addSphere(geomSize[0], MJC_MASS_FROM_XML, i, j);
          break;
        }
        case mjGEOM_CAPSULE: {
          // geomsize = (radius_, height * 0.5)
          addCapsule(geomSize[0], geomSize[1] * 2, MJC_MASS_FROM_XML, i, j);
          break;
        }
        case mjGEOM_BOX: {
          // geomsize = (xlength, ylength, zlength)
          addBox(geomSize[0] * 2, geomSize[1] * 2, geomSize[2] * 2, MJC_MASS_FROM_XML, i, j);
          break;
        }
        case mjGEOM_CYLINDER: {
          // geomsize = (radius_, height * 0.5)
          addCylinder(geomSize[0], geomSize[1] * 2, MJC_MASS_FROM_XML, i, j);
          break;
        }
        case mjGEOM_ELLIPSOID: {
          RAIFATAL("ellpisoid geometry is not supported.")
          break;
        }
        case mjGEOM_NONE: {
          RAIFATAL("invalid geometry type")
          break;
        }
        default: {
          RAIFATAL("not supported geometry type");
        }
      }
    }
  }
}

mujoco_sim::MjcWorld_RG::~MjcWorld_RG() {

}

benchmark::SingleBodyHandle mujoco_sim::MjcWorld_RG::addSphere(double radius,
                                                            double mass,
                                                            int bodyId,
                                                            int geomId) {
  benchmark::SingleBodyHandle handle(world_.addSphere(radius, mass, bodyId, geomId), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::MjcWorld_RG::addBox(double xLength,
                                                         double yLength,
                                                         double zLength,
                                                         double mass,
                                                         int bodyId,
                                                         int geomId) {
  benchmark::SingleBodyHandle handle(world_.addBox(xLength, yLength, zLength, mass, bodyId, geomId), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Box(xLength, yLength, zLength, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::MjcWorld_RG::addCheckerboard(double gridSize,
                                                                  double xLength,
                                                                  double yLength,
                                                                  double reflectanceI,
                                                                  bo::CheckerboardShape shape,
                                                                  int bodyId,
                                                                  int geomId,
                                                                  int flags) {
  benchmark::SingleBodyHandle handle(world_.addCheckerboard(gridSize, xLength, yLength, reflectanceI, shape, bodyId, geomId), {}, {});
  handle.hidable = false;
  if(gui_) {
    handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
    static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & bo::GRID;
    gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
  }
  sbHandles_.push_back(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::MjcWorld_RG::addCapsule(double radius,
                                                             double height,
                                                             double mass,
                                                             int bodyId,
                                                             int geomid) {
  benchmark::SingleBodyHandle handle(world_.addCapsule(radius, height, mass, bodyId, geomid), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::MjcWorld_RG::addCylinder(double radius,
                                                              double height,
                                                              double mass,
                                                              int bodyId,
                                                              int geomId) {
  benchmark::SingleBodyHandle handle(world_.addCylinder(radius, height, mass, bodyId, geomId), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Cylinder(radius, height, true));
  processSingleBody(handle);
  return handle;
}

void mujoco_sim::MjcWorld_RG::setGravity(Eigen::Vector3d gravity) {
  world_.setGravity({gravity[0], gravity[1], gravity[2]});
}

void mujoco_sim::MjcWorld_RG::setERP(double erp, double erp2, double frictionErp) {
  RAIFATAL("no erp for mujoco")
}

benchmark::SingleBodyHandle mujoco_sim::MjcWorld_RG::getSingleBodyHandle(int index) {
  if(index > sbHandles_.size())
  RAIFATAL("get singlebody handle failed. invalid index");
  return sbHandles_[index];
}
const mujoco_sim::EigenVec mujoco_sim::MjcWorld_RG::getGeneralizedCoordinate() {
  return world_.getGeneralizedCoordinate();
}
const mujoco_sim::EigenVec mujoco_sim::MjcWorld_RG::getGeneralizedVelocity() {
  return world_.getGeneralizedVelocity();
}
void mujoco_sim::MjcWorld_RG::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  world_.setGeneralizedCoordinate(jointState);
}
void mujoco_sim::MjcWorld_RG::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  world_.setGeneralizedVelocity(jointVel);
}
void mujoco_sim::MjcWorld_RG::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  world_.setGeneralizedCoordinate(jointState);
}
void mujoco_sim::MjcWorld_RG::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  world_.setGeneralizedVelocity(jointVel);
}
void mujoco_sim::MjcWorld_RG::setGeneralizedForce(std::initializer_list<double> tau) {
  world_.setGeneralizedForce(tau);
}
void mujoco_sim::MjcWorld_RG::setGeneralizedForce(const Eigen::VectorXd &tau) {
  world_.setGeneralizedForce(tau);
}
void mujoco_sim::MjcWorld_RG::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
  world_.getState(genco, genvel);
}
void mujoco_sim::MjcWorld_RG::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
  world_.setState(genco, genvel);
}
const mujoco_sim::EigenVec mujoco_sim::MjcWorld_RG::getGeneralizedForce() {
  return world_.getGeneralizedForce();
}
int mujoco_sim::MjcWorld_RG::getDOF() {
  return world_.getDOF();
}
int mujoco_sim::MjcWorld_RG::getStateDimension() {
  return world_.getGeneralizedCoordinateDim();
}

int mujoco_sim::MjcWorld_RG::getWorldNumContacts() {
  return world_.getWorldNumContacts();
}
int mujoco_sim::MjcWorld_RG::getNumObject() {
  return world_.getNumObject();
}

void mujoco_sim::MjcWorld_RG::setNoSlipParameter(int maxiter) {
  world_.setNoSlipParameter(maxiter);
}

void mujoco_sim::MjcWorld_RG::setTimeStep(double timeStep) {
  timeStep_ = timeStep;
  world_.setTimeStep(timeStep);
}

void mujoco_sim::MjcWorld_RG::loop(double realTimeFactor) {
  while (visualizerLoop(timeStep_, realTimeFactor))
    integrate();
}

void mujoco_sim::MjcWorld_RG::integrate() {
  world_.integrate();
}

void mujoco_sim::MjcWorld_RG::integrate1() {
  world_.integrate1();
}

void mujoco_sim::MjcWorld_RG::integrate2() {
  world_.integrate2();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1>> mujoco_sim::MjcWorld_RG::getLinearMomentumInCartesianSpace() {
  return world_.getLinearMomentumInCartesianSpace();
}

double mujoco_sim::MjcWorld_RG::getTotalMass() {
  return world_.getTotalMass();
}


double mujoco_sim::MjcWorld_RG::getEnergy(const benchmark::Vec<3> &gravity) {
  return world_.getEnergy(gravity);
}

void mujoco_sim::MjcWorld_RG::forwardKinematics() {
  world_.forwardKinematics();
}

void mujoco_sim::MjcWorld_RG::loop(double dt, double realTimeFactor) {
  RAIFATAL("use setTimeStep(double dt) + loop() instead")
}

void mujoco_sim::MjcWorld_RG::integrate(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate() instead")
}
void mujoco_sim::MjcWorld_RG::integrate1(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate1() instead")
}
void mujoco_sim::MjcWorld_RG::integrate2(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate2() instead")
}

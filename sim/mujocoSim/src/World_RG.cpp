//
// Created by kangd on 18.02.18.
//

#include "World_RG.hpp"

// the mass data is from XML
#define MJC_MASS_FROM_XML -1

mujoco_sim::World_RG::World_RG(int windowWidth,
                               int windowHeight,
                               float cms,
                               const char *modelPath,
                               const char *keyPath,
                               int flags,
                               mujoco_sim::SolverOption solverOption) :
    world_(modelPath, keyPath, solverOption),
    benchmark::World_RG(windowWidth, windowHeight, cms, flags) {
  initFromModel();
}

mujoco_sim::World_RG::World_RG(const char *modelPath,
                               const char *keyPath,
                               mujoco_sim::SolverOption solverOption) :
    world_(modelPath, keyPath, solverOption),
    benchmark::World_RG() {
  initFromModel();
}

void mujoco_sim::World_RG::initFromModel() {

  mjModel *model = world_.getWorldModel();

  // make objects
  for(int i = 0; i < model->nbody; i++) {
    const int geomNumInBody = model->body_geomnum[i];
    const int geomAddrInBody = model->body_geomadr[i];

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

mujoco_sim::World_RG::~World_RG() {

}

benchmark::SingleBodyHandle mujoco_sim::World_RG::addSphere(double radius,
                                                            double mass,
                                                            int bodyId,
                                                            int geomId) {
  benchmark::SingleBodyHandle handle(world_.addSphere(radius, mass, bodyId, geomId), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::World_RG::addBox(double xLength,
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

benchmark::SingleBodyHandle mujoco_sim::World_RG::addCheckerboard(double gridSize,
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

benchmark::SingleBodyHandle mujoco_sim::World_RG::addCapsule(double radius,
                                                             double height,
                                                             double mass,
                                                             int bodyId,
                                                             int geomid) {
  benchmark::SingleBodyHandle handle(world_.addCapsule(radius, height, mass, bodyId, geomid), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::World_RG::addCylinder(double radius,
                                                              double height,
                                                              double mass,
                                                              int bodyId,
                                                              int geomId) {
  benchmark::SingleBodyHandle handle(world_.addCylinder(radius, height, mass, bodyId, geomId), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Cylinder(radius, height, true));
  processSingleBody(handle);
  return handle;
}

void mujoco_sim::World_RG::integrate(double dt) {
  world_.integrate(dt);
}

void mujoco_sim::World_RG::setGravity(Eigen::Vector3d gravity) {
  world_.setGravity({gravity[0], gravity[1], gravity[2]});
}

void mujoco_sim::World_RG::setERP(double erp, double erp2, double frictionErp) {}
benchmark::SingleBodyHandle mujoco_sim::World_RG::getSingleBodyHandle(int index) {
  if(index > sbHandles_.size())
  RAIFATAL("get singlebody handle failed. invalid index");
  return sbHandles_[index];
}
const mujoco_sim::EigenVec mujoco_sim::World_RG::getGeneralizedCoordinate() {
  return world_.getGeneralizedCoordinate();
}
const mujoco_sim::EigenVec mujoco_sim::World_RG::getGeneralizedVelocity() {
  return world_.getGeneralizedVelocity();
}
void mujoco_sim::World_RG::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  world_.setGeneralizedCoordinate(jointState);
}
void mujoco_sim::World_RG::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  world_.setGeneralizedVelocity(jointVel);
}
void mujoco_sim::World_RG::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  world_.setGeneralizedCoordinate(jointState);
}
void mujoco_sim::World_RG::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  world_.setGeneralizedVelocity(jointVel);
}
void mujoco_sim::World_RG::setGeneralizedForce(std::initializer_list<double> tau) {
  world_.setGeneralizedForce(tau);
}
void mujoco_sim::World_RG::setGeneralizedForce(const Eigen::VectorXd &tau) {
  world_.setGeneralizedForce(tau);
}
void mujoco_sim::World_RG::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
  world_.getState(genco, genvel);
}
void mujoco_sim::World_RG::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
  world_.setState(genco, genvel);
}
const mujoco_sim::EigenVec mujoco_sim::World_RG::getGeneralizedForce() {
  return world_.getGeneralizedForce();
}
int mujoco_sim::World_RG::getDOF() {
  return world_.getDOF();
}
int mujoco_sim::World_RG::getGeneralizedCoordinateDim() {
  return world_.getGeneralizedCoordinateDim();
}

int mujoco_sim::World_RG::getWorldNumContacts() {
  return world_.getWorldNumContacts();
}
int mujoco_sim::World_RG::getNumObject() {
  return world_.getNumObject();
}


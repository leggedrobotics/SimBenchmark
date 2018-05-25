//
// Created by kangd on 25.05.18.
//

#include "BtMbArticulatedSystem.hpp"

namespace bullet_multibody_sim {

object::BtMbArticulatedSystem::BtMbArticulatedSystem(std::string filePath,
                                                     ObjectFileType fileType,
                                                     b3RobotSimulatorClientAPI_NoGUI *api) : api_(api) {

  int objectId = -1;

  // load model file
  switch (fileType) {
    case URDF:
      objectId = api->loadURDF(filePath);
      break;
    case SDF:
    case MJCF:
    RAIFATAL("currently, only URDF is supported")
      break;
    default:
    RAIFATAL("wrong object file type")
      break;
  }

  RAIFATAL_IF(objectId == -1, "cannot load Object")

  // initialize visual object data

}

object::BtMbArticulatedSystem::~BtMbArticulatedSystem() {}

const benchmark::object::ArticulatedSystemInterface::EigenVec object::BtMbArticulatedSystem::getGeneralizedCoordinate() {
  return nullptr;
}

const benchmark::object::ArticulatedSystemInterface::EigenVec object::BtMbArticulatedSystem::getGeneralizedVelocity() {
  return nullptr;
}

void object::BtMbArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {

}

void object::BtMbArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {

}

void object::BtMbArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState) {

}

void object::BtMbArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel) {

}

void object::BtMbArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau) {
}

void object::BtMbArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
}

void object::BtMbArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
}

void object::BtMbArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau) {
}

const benchmark::object::ArticulatedSystemInterface::EigenVec object::BtMbArticulatedSystem::getGeneralizedForce() {
  return nullptr;
}

int object::BtMbArticulatedSystem::getDOF() {
  return 0;
}

int object::BtMbArticulatedSystem::getStateDimension() {
  return 0;
}

const Eigen::Map<Eigen::Matrix<double, 3, 1>> object::BtMbArticulatedSystem::getLinearMomentumInCartesianSpace() {
  return nullptr;
}

double object::BtMbArticulatedSystem::getTotalMass() {
  return 0;
}
double object::BtMbArticulatedSystem::getEnergy(const benchmark::Vec<3> &gravity) {
  return 0;
}
} // bullet_multibody_sim
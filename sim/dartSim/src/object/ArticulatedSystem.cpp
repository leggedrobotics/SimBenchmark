//
// Created by kangd on 04.04.18.
//

#include "ArticulatedSystem.hpp"

namespace dart_sim {
namespace object {

ArticulatedSystem::ArticulatedSystem(std::string urdfFile) {

  dart::utils::DartLoader urdfLoader;
  skeletonPtr_ = urdfLoader.parseSkeleton(urdfFile);

  if(!skeletonPtr_)
    RAIFATAL("cannot load articulated system from URDF")

  dof_ = (int)skeletonPtr_->getNumDofs();

  genVelocity_.resize(dof_);
  genVelocity_.setZero();
  genForce_.resize(dof_);
  genForce_.setZero();
}

ArticulatedSystem::~ArticulatedSystem() {

}

const benchmark::object::ArticulatedSystemInterface::EigenVec ArticulatedSystem::getGeneralizedCoordinate() {
  Eigen::VectorXd pos = skeletonPtr_->getPositions();
  for(int i = 0; i < dof_; i++) {
    genCoordinate_[i] = pos[i];
  }
  return genCoordinate_.e();
}

const benchmark::object::ArticulatedSystemInterface::EigenVec ArticulatedSystem::getGeneralizedVelocity() {
  Eigen::VectorXd vel = skeletonPtr_->getVelocities();
  for(int i = 0; i < dof_; i++) {
    genVelocity_[i] = vel[i];
  }
  return genVelocity_.e();
}

const benchmark::object::ArticulatedSystemInterface::EigenVec ArticulatedSystem::getGeneralizedForce() {
  Eigen::VectorXd force = skeletonPtr_->getForces();
  for(int i = 0; i < dof_; i++) {
    genForce_[i] = force[i];
  }
  return genForce_.e();
}

void ArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {

}
void ArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {

}
void ArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState) {

}
void ArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel) {

}
void ArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau) {

}
void ArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {

}
void ArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {

}
void ArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau) {

}

int ArticulatedSystem::getDOF() {
  return dof_;
}

} // object
} // dart_sim

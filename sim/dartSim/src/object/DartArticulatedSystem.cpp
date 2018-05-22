//
// Created by kangd on 04.04.18.
//

#include "DartArticulatedSystem.hpp"

namespace dart_sim {
namespace object {

DartArticulatedSystem::DartArticulatedSystem(std::string urdfFile) {

  urdfFile += "robot.urdf";
  dart::utils::DartLoader urdfLoader;
  skeletonPtr_ = urdfLoader.parseSkeleton(urdfFile);
  RAIFATAL_IF(!skeletonPtr_, "cannot load articulated system from URDF")

  // articulated system initializing
  init();
}

DartArticulatedSystem::~DartArticulatedSystem() {

}

void DartArticulatedSystem::init() {

  dof_ = (int)skeletonPtr_->getNumDofs();

  if(skeletonPtr_->getRootJoint()->getNumDofs() == 0) {
    // fixed base
    isFixed_ = true;
  stateDimension_ = dof_;
  }
  else {
    // floating base
    isFixed_ = false;
    stateDimension_ = dof_ + 1;
  }

  genCoordinate_.resize(stateDimension_);
  genCoordinate_.setZero();
  genVelocity_.resize(dof_);
  genVelocity_.setZero();
  genForce_.resize(dof_);
  genForce_.setZero();

  // init visual recursively
  initVisual(skeletonPtr_->getRootBodyNode());
}

void DartArticulatedSystem::updateVisuals() {
  visObj.clear();
  visColObj.clear();
  visProps_.clear();
  visColProps_.clear();

  initVisual(skeletonPtr_->getRootBodyNode());
}

void DartArticulatedSystem::initVisual(dart::dynamics::BodyNode *body) {

  /// visual node
  for(int i = 0; i < body->getNumShapeNodesWith<dart::dynamics::VisualAspect>(); i++) {

    // shape
    dart::dynamics::ShapeNodePtr shapeNodePtr =
        body->getShapeNodesWith<dart::dynamics::VisualAspect>()[i];
    dart::dynamics::ShapePtr shape = shapeNodePtr->getShape();

    Eigen::Isometry3d tf = shapeNodePtr->getWorldTransform();

    // orientation
    benchmark::Mat<3, 3> mat;
    Eigen::Matrix3d rotMat = tf.linear();
    mat.e() = rotMat;

    // position
    benchmark::Vec<3> position = {tf.translation().x(),
                                  tf.translation().y(),
                                  tf.translation().z()};

    if(shape->is<dart::dynamics::BoxShape>()) {
      // box
      Eigen::Vector3d size = std::static_pointer_cast<dart::dynamics::BoxShape>(shape)->getSize();
      benchmark::Vec<4> boxSize;
      boxSize = {
          size[0], size[1], size[2]
      };

      visObj.emplace_back(std::make_tuple(mat, position, i, benchmark::object::Shape::Box, color_));
      visProps_.emplace_back(std::make_pair("", boxSize));
    }
    else if(shape->is<dart::dynamics::SphereShape>()) {
      // sphere
      double radius = std::static_pointer_cast<dart::dynamics::SphereShape>(shape)->getRadius();
      benchmark::Vec<4> sphereSize;
      sphereSize = {
          radius, 0, 0
      };

      visObj.emplace_back(std::make_tuple(mat, position, i, benchmark::object::Shape::Sphere, color_));
      visProps_.emplace_back(std::make_pair("", sphereSize));
    }
    else if(shape->is<dart::dynamics::CylinderShape>()) {
      // cylinder
      double radius = std::static_pointer_cast<dart::dynamics::CylinderShape>(shape)->getRadius();
      double height = std::static_pointer_cast<dart::dynamics::CylinderShape>(shape)->getHeight();
      benchmark::Vec<4> cylSize;
      cylSize = {
          radius, height, 0
      };

      visObj.emplace_back(std::make_tuple(mat, position, i, benchmark::object::Shape::Cylinder, color_));
      visProps_.emplace_back(std::make_pair("", cylSize));
    }
    else if(shape->is<dart::dynamics::MeshShape>()) {
      // mesh
      Eigen::Vector3d scale = std::static_pointer_cast<dart::dynamics::MeshShape>(shape)->getScale();
      std::string meshPath = std::static_pointer_cast<dart::dynamics::MeshShape>(shape)->getMeshPath();
      benchmark::Vec<4> meshSize;
      meshSize = {
          scale[0], scale[1], scale[2]
      };

      visObj.emplace_back(std::make_tuple(mat, position, i, benchmark::object::Shape::Mesh, color_));
      visProps_.emplace_back(std::make_pair(meshPath, meshSize));
    }
    else {
      // else
      RAIFATAL("not supported shape")
    }
  }

  /// collision node
  for(int i = 0; i < body->getNumShapeNodesWith<dart::dynamics::CollisionAspect>(); i++) {

    // shape
    dart::dynamics::ShapeNodePtr shapeNodePtr =
        body->getShapeNodesWith<dart::dynamics::CollisionAspect>()[i];
    dart::dynamics::ShapePtr shape = shapeNodePtr->getShape();

    Eigen::Isometry3d tf = shapeNodePtr->getWorldTransform();

    // orientation
    benchmark::Mat<3, 3> mat;
    Eigen::Matrix3d rotMat = tf.linear();
    mat.e() = rotMat;

    // position
    benchmark::Vec<3> position = {tf.translation().x(),
                                  tf.translation().y(),
                                  tf.translation().z()};

    if(shape->is<dart::dynamics::BoxShape>()) {
      // box
      Eigen::Vector3d size = std::static_pointer_cast<dart::dynamics::BoxShape>(shape)->getSize();
      benchmark::Vec<4> boxSize;
      boxSize = {
          size[0], size[1], size[2]
      };

      visColObj.emplace_back(std::make_tuple(mat, position, i, benchmark::object::Shape::Box));
      visColProps_.emplace_back(std::make_pair("", boxSize));
    }
    else if(shape->is<dart::dynamics::SphereShape>()) {
      // sphere
      double radius = std::static_pointer_cast<dart::dynamics::SphereShape>(shape)->getRadius();
      benchmark::Vec<4> sphereSize;
      sphereSize = {
          radius, 0, 0
      };

      visColObj.emplace_back(std::make_tuple(mat, position, i, benchmark::object::Shape::Sphere));
      visColProps_.emplace_back(std::make_pair("", sphereSize));
    }
    else if(shape->is<dart::dynamics::CylinderShape>()) {
      // cylinder
      double radius = std::static_pointer_cast<dart::dynamics::CylinderShape>(shape)->getRadius();
      double height = std::static_pointer_cast<dart::dynamics::CylinderShape>(shape)->getHeight();
      benchmark::Vec<4> cylSize;
      cylSize = {
          radius, height, 0
      };

      visColObj.emplace_back(std::make_tuple(mat, position, i, benchmark::object::Shape::Cylinder));
      visColProps_.emplace_back(std::make_pair("", cylSize));
    }
    else {
      // else
      RAIFATAL("not supported shape")
    }
  }

  for(int i = 0; i < body->getNumChildBodyNodes(); i++) {
    initVisual(body->getChildBodyNode(i));
  }
}

const benchmark::object::ArticulatedSystemInterface::EigenVec DartArticulatedSystem::getGeneralizedCoordinate() {
  Eigen::VectorXd pos = skeletonPtr_->getPositions();
  if(isFixed_){
    // fixed base
    for(int i = 0; i < stateDimension_; i++) {
      genCoordinate_[i] = pos[i];
    }
    return genCoordinate_.e();
  } else  {
    // floating base
    Eigen::Quaterniond quaternion = dart::math::expToQuat(Eigen::Vector3d(pos[0], pos[1], pos[2]));

    // rotation
    genCoordinate_[3] = quaternion.w();
    genCoordinate_[4] = quaternion.x();
    genCoordinate_[5] = quaternion.y();
    genCoordinate_[6] = quaternion.z();

    // position
    genCoordinate_[0] = pos[3];
    genCoordinate_[1] = pos[4];
    genCoordinate_[2] = pos[5];

    for(int i = 7; i < stateDimension_; i++) {
      genCoordinate_[i] = pos[i-1];
    }
    return genCoordinate_.e();
  }
}

const benchmark::object::ArticulatedSystemInterface::EigenVec DartArticulatedSystem::getGeneralizedVelocity() {
  Eigen::VectorXd vel = skeletonPtr_->getVelocities();
  if(isFixed_) {
    for(int i = 0; i < dof_; i++) {
      genVelocity_[i] = vel[i];
    }
  } else {
    genVelocity_[0] = vel[3];
    genVelocity_[1] = vel[4];
    genVelocity_[2] = vel[5];
    genVelocity_[3] = vel[0];
    genVelocity_[4] = vel[1];
    genVelocity_[5] = vel[2];

    for(int i = 6; i < dof_; i++) {
      genVelocity_[i] = vel[i];
    }
  };
  return genVelocity_.e();
}

const benchmark::object::ArticulatedSystemInterface::EigenVec DartArticulatedSystem::getGeneralizedForce() {
  Eigen::VectorXd force = skeletonPtr_->getForces();
  if(isFixed_) {
    for(int i = 0; i < dof_; i++) {
      genForce_[i] = force[i];
    }
  } else {
    Eigen::Vector6d baseForce = skeletonPtr_->getBodyNode(0)->getExternalForceGlobal();
    genForce_[0] = baseForce[3];
    genForce_[1] = baseForce[4];
    genForce_[2] = baseForce[5];
    genForce_[3] = baseForce[0];
    genForce_[4] = baseForce[1];
    genForce_[5] = baseForce[2];

    for(int i = 6; i < dof_; i++) {
      genForce_[i] = force[i];
    }
  };
  return genForce_.e();
}

void DartArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  RAIFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input")
  if(isFixed_){
    // fixed base
    for(int i = 0; i < stateDimension_; i++) {
      genCoordinate_[i] = jointState[i];
      skeletonPtr_->setPosition(i, jointState[i]);
    }
  } else  {
    // floating base
    Eigen::Vector3d exp = dart::math::quatToExp({jointState[3],
                                                 jointState[4],
                                                 jointState[5],
                                                 jointState[6]});
    // rotation
    skeletonPtr_->setPosition(0, exp[0]);
    skeletonPtr_->setPosition(1, exp[1]);
    skeletonPtr_->setPosition(2, exp[2]);

    // position
    skeletonPtr_->setPosition(3, jointState[0]);
    skeletonPtr_->setPosition(4, jointState[1]);
    skeletonPtr_->setPosition(5, jointState[2]);

    for(int i = 0; i < stateDimension_; i++) {
      genCoordinate_[i] = jointState[i];

      if(i > 6) {
        // joint pos
        skeletonPtr_->setPosition(i-1, jointState[i]);
      }
    }
  }
}

void DartArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  RAIFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input")
  if(isFixed_){
    // fixed base
    for(int i = 0; i < stateDimension_; i++) {
      genCoordinate_[i] = jointState.begin()[i];
      skeletonPtr_->setPosition(i, jointState.begin()[i]);
    }
  } else  {
    // floating base
    Eigen::Vector3d exp = dart::math::quatToExp({jointState.begin()[3],
                                                 jointState.begin()[4],
                                                 jointState.begin()[5],
                                                 jointState.begin()[6]});
    // rotation
    skeletonPtr_->setPosition(0, exp[0]);
    skeletonPtr_->setPosition(1, exp[1]);
    skeletonPtr_->setPosition(2, exp[2]);

    // position
    skeletonPtr_->setPosition(3, jointState.begin()[0]);
    skeletonPtr_->setPosition(4, jointState.begin()[1]);
    skeletonPtr_->setPosition(5, jointState.begin()[2]);

    for(int i = 0; i < stateDimension_; i++) {
      genCoordinate_[i] = jointState.begin()[i];

      if(i > 6) {
        // joint pos
        skeletonPtr_->setPosition(i-1, jointState.begin()[i]);
      }
    }
  }
}

void DartArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  if(isFixed_)
    skeletonPtr_->setVelocities(jointVel);
  else {
    skeletonPtr_->setVelocity(0, jointVel[3]);
    skeletonPtr_->setVelocity(1, jointVel[4]);
    skeletonPtr_->setVelocity(2, jointVel[5]);
    skeletonPtr_->setVelocity(3, jointVel[0]);
    skeletonPtr_->setVelocity(4, jointVel[1]);
    skeletonPtr_->setVelocity(5, jointVel[2]);
    for(int i = 6; i < dof_; i++) {
      skeletonPtr_->setVelocity(i, jointVel[i]);
    }
  }

  for(int i = 0; i < dof_; i++) {
    genVelocity_[i] = jointVel[i];
  }
}

void DartArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  if(isFixed_) {
    for(int i = 0; i < dof_; i++) {
      skeletonPtr_->setVelocity(i, jointVel.begin()[i]);
    }
  }
  else {
    skeletonPtr_->setVelocity(0, jointVel.begin()[3]);
    skeletonPtr_->setVelocity(1, jointVel.begin()[4]);
    skeletonPtr_->setVelocity(2, jointVel.begin()[5]);
    skeletonPtr_->setVelocity(3, jointVel.begin()[0]);
    skeletonPtr_->setVelocity(4, jointVel.begin()[1]);
    skeletonPtr_->setVelocity(5, jointVel.begin()[2]);
    for(int i = 6; i < dof_; i++) {
      skeletonPtr_->setVelocity(i, jointVel.begin()[i]);
    }
  }

  for(int i = 0; i < dof_; i++) {
    genVelocity_[i] = jointVel.begin()[i];
  }
}

void DartArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  if(isFixed_) {
    for(int i = 0; i < dof_; i++) {
      skeletonPtr_->setForce(i, tau.begin()[i]);
    }
  }
  else {
    Eigen::Vector3d force(tau.begin()[0], tau.begin()[1], tau.begin()[2]);
    Eigen::Vector3d torque(tau.begin()[3], tau.begin()[4], tau.begin()[5]);
    skeletonPtr_->getBodyNode(0)->setExtForce(force);
    skeletonPtr_->getBodyNode(0)->setExtTorque(torque);
    for(int i = 6; i < dof_; i++) {
      skeletonPtr_->setForce(i, tau.begin()[i]);
    }
  }

  for(int i = 0; i < dof_; i++) {
    genForce_[i] = tau.begin()[i];
  }
}

void DartArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  if(isFixed_) {
    for(int i = 0; i < dof_; i++) {
      skeletonPtr_->setForce(i, tau[i]);
    }
  }
  else {
    Eigen::Vector3d force(tau[0], tau[1], tau[2]);
    Eigen::Vector3d torque(tau[3], tau[4], tau[5]);
    skeletonPtr_->getBodyNode(0)->setExtForce(force);
    skeletonPtr_->getBodyNode(0)->setExtTorque(torque);
    for(int i = 6; i < dof_; i++) {
      skeletonPtr_->setForce(i, tau[i]);
    }
  }

  for(int i = 0; i < dof_; i++) {
    genForce_[i] = tau[i];
  }
}

void DartArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
  genco = getGeneralizedCoordinate();
  genvel = getGeneralizedVelocity();
}
void DartArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
  setGeneralizedCoordinate(genco);
  setGeneralizedVelocity(genvel);
}
int DartArticulatedSystem::getDOF() {
  return dof_;
}
void DartArticulatedSystem::setColor(Eigen::Vector4d color) {
  color_ = {
      color[0], color[1], color[2], color[3]};
}
int DartArticulatedSystem::getStateDimension() {
  return stateDimension_;
}

const Eigen::Map<Eigen::Matrix<double, 3, 1>> DartArticulatedSystem::getLinearMomentumInCartesianSpace() {
  double mass = skeletonPtr_->getMass();
  Eigen::Vector3d comvel = skeletonPtr_->getCOMLinearVelocity();

  linearMomentum_ = {mass * comvel.x(),
                     mass * comvel.y(),
                     mass * comvel.z()};
  return linearMomentum_.e();
}

double DartArticulatedSystem::getTotalMass() {
  return skeletonPtr_->getMass();
}

double DartArticulatedSystem::getEnergy(const benchmark::Vec<3> &gravity) {
  return skeletonPtr_->computePotentialEnergy() + skeletonPtr_->computeKineticEnergy();
}

void DartArticulatedSystem::setInternalCollision(bool Yn) {
  skeletonPtr_->setSelfCollisionCheck(Yn);
}

} // object
} // dart_sim

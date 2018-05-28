//
// Created by kangd on 26.05.18.
//

#include "BtMbSingleBodyObject.hpp"

namespace bullet_mb_sim {
namespace object {

BtMbSingleBodyObject::BtMbSingleBodyObject(b3RobotSimulatorClientAPI_NoGUI *api) : api_(api) {
}

const benchmark::eQuaternion BtMbSingleBodyObject::getQuaternion() {
  getBulletQuaternion();
  return quatTemp_.e();
}

void BtMbSingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
  getBulletQuaternion();
  quat = quatTemp_;
}

const benchmark::eRotationMat BtMbSingleBodyObject::getRotationMatrix() {
  getBulletQuaternion();
  benchmark::quatToRotMat(quatTemp_, rotMatTemp_);
  return rotMatTemp_.e();
}

void BtMbSingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
  getBulletQuaternion();
  benchmark::quatToRotMat(quatTemp_, rotMatTemp_);
  rotation = rotMatTemp_;
}

const benchmark::eVector3 BtMbSingleBodyObject::getPosition() {
  getBulletPosition();
  return posTemp_.e();
}

const benchmark::eVector3 BtMbSingleBodyObject::getComPosition() {
  getBulletPosition();
  return posTemp_.e();
}

const benchmark::eVector3 BtMbSingleBodyObject::getLinearVelocity() {
  getBulletLinearVelocity();
  return linVelTemp_.e();
}

const benchmark::eVector3 BtMbSingleBodyObject::getAngularVelocity() {
  getBulletAngularVelocity();
  return angVelTemp_.e();
}

void BtMbSingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
  getBulletPosition();
  pos_w = posTemp_;
}

void BtMbSingleBodyObject::setPosition(Eigen::Vector3d originPosition) {

}

void BtMbSingleBodyObject::setPosition(double x, double y, double z) {

}

void BtMbSingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {

}

void BtMbSingleBodyObject::setOrientation(double w, double x, double y, double z) {

}

void BtMbSingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {

}

void BtMbSingleBodyObject::setOrientationRandom() {

}

void BtMbSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {

}

void BtMbSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {

}

void BtMbSingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {

}

void BtMbSingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {

}

void BtMbSingleBodyObject::setExternalForce(Eigen::Vector3d force) {

}

void BtMbSingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {

}

void BtMbSingleBodyObject::setRestitutionCoefficient(double restitution) {

}

void BtMbSingleBodyObject::setFrictionCoefficient(double friction) {
  b3RobotSimulatorChangeDynamicsArgs arg;
  arg.m_lateralFriction = friction;
  api_->changeDynamics(objectId_, -1, arg);
}

double BtMbSingleBodyObject::getKineticEnergy() {
  return 0;
}

double BtMbSingleBodyObject::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
  return 0;
}

double BtMbSingleBodyObject::getEnergy(const benchmark::Vec<3> &gravity) {
  return 0;
}

const Eigen::Map<Eigen::Matrix<double, 3, 1>> BtMbSingleBodyObject::getLinearMomentum() {
  getBulletLinearVelocity();
  benchmark::vecScalarMul(mass_, linVelTemp_, linearMomentum_);
  return linearMomentum_.e();
}

bool BtMbSingleBodyObject::isVisualizeFramesAndCom() const {
  return false;
}

void BtMbSingleBodyObject::getBulletQuaternion() {
  btVector3 bPos;
  btQuaternion bQuat;
  api_->getBasePositionAndOrientation(objectId_, bPos, bQuat);
  quatTemp_ = {
      bQuat.w(),   // w
      bQuat.x(),   // x
      bQuat.y(),   // y
      bQuat.z(),   // z
  };
}

void BtMbSingleBodyObject::getBulletPosition() {
  btVector3 bPos;
  btQuaternion bQuat;
  api_->getBasePositionAndOrientation(objectId_, bPos, bQuat);
  posTemp_ = {
      bPos.x(),   // x
      bPos.y(),   // y
      bPos.z(),   // z
  };
}

void BtMbSingleBodyObject::getBulletLinearVelocity() {
  btVector3 bLinVel;
  btVector3 bAngVel;
  api_->getBaseVelocity(objectId_, bLinVel, bAngVel);
  posTemp_ = {
      bLinVel.x(),   // x
      bLinVel.y(),   // y
      bLinVel.z(),   // z
  };
}

void BtMbSingleBodyObject::getBulletAngularVelocity() {
  btVector3 bLinVel;
  btVector3 bAngVel;
  api_->getBaseVelocity(objectId_, bLinVel, bAngVel);
  posTemp_ = {
      bAngVel.x(),   // x
      bAngVel.y(),   // y
      bAngVel.z(),   // z
  };
}

} // object
} // bullet_mb_sim
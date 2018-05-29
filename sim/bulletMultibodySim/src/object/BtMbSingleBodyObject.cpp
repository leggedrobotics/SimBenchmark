//
// Created by kangd on 26.05.18.
//

#include "BtMbSingleBodyObject.hpp"

namespace bullet_mb_sim {
namespace object {

BtMbSingleBodyObject::BtMbSingleBodyObject(double mass, b3RobotSimulatorClientAPI_NoGUI *api)
    : api_(api), mass_(mass) {}

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
  posTemp_.e() = originPosition;
  getBulletQuaternion();
  setBulletPositionAndQuaternion();
}

void BtMbSingleBodyObject::setPosition(double x, double y, double z) {
  posTemp_ = {x, y, z};
  getBulletQuaternion();
  setBulletPositionAndQuaternion();
}

void BtMbSingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {
  quatTemp_ = {
      quaternion.w(),
      quaternion.x(),
      quaternion.y(),
      quaternion.z()
  };
  getBulletPosition();
  setBulletPositionAndQuaternion();
}

void BtMbSingleBodyObject::setOrientation(double w, double x, double y, double z) {
  quatTemp_ = {w, x, y, z};
  getBulletPosition();
  setBulletPositionAndQuaternion();
}

void BtMbSingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {
  rotMatTemp_.e() = rotationMatrix;
  benchmark::rotMatToQuat(rotMatTemp_, quatTemp_);
  getBulletPosition();
  setBulletPositionAndQuaternion();
}

void BtMbSingleBodyObject::setOrientationRandom() {
  RAIFATAL("setOrientationRandom not implemented yet")
}

void BtMbSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {
  posTemp_.e() = originPosition;
  quatTemp_ = {
      quaternion.w(),
      quaternion.x(),
      quaternion.y(),
      quaternion.z()
  };
  setBulletPositionAndQuaternion();
}

void BtMbSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {
  posTemp_.e() = originPosition;
  rotMatTemp_.e() = rotationMatrix;
  benchmark::rotMatToQuat(rotMatTemp_, quatTemp_);
  setBulletPositionAndQuaternion();
}

void BtMbSingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {
  linVelTemp_.e() = linearVelocity;
  angVelTemp_.e() = angularVelocity;
  setBulletLinearAndAngularVelocity();
}

void BtMbSingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
  linVelTemp_ = {dx, dy, dz};
  angVelTemp_ = {wx, wy, wz};
  setBulletLinearAndAngularVelocity();
}

void BtMbSingleBodyObject::setExternalForce(Eigen::Vector3d force) {
  btVector3 bForce(
      force[0],
      force[1],
      force[2]
  );
  btVector3 bPosition(0, 0, 0);
  api_->applyExternalForce(objectId_, -1, bForce, bPosition, EF_WORLD_FRAME);
}

void BtMbSingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  btVector3 bTorque(
      torque[0],
      torque[1],
      torque[2]
  );
  btVector3 bPosition(0, 0, 0);
  api_->applyExternalTorque(objectId_, -1, bTorque, EF_WORLD_FRAME);
}

void BtMbSingleBodyObject::setRestitutionCoefficient(double restitution) {
  b3RobotSimulatorChangeDynamicsArgs arg;
  arg.m_restitution = restitution;
  api_->changeDynamics(objectId_, -1, arg);
}

void BtMbSingleBodyObject::setFrictionCoefficient(double friction) {
  b3RobotSimulatorChangeDynamicsArgs arg;
  arg.m_lateralFriction = friction;
  api_->changeDynamics(objectId_, -1, arg);
}

double BtMbSingleBodyObject::getKineticEnergy() {
  getBulletLinearVelocity();
  getBulletAngularVelocity();

  // ang
  double angEnergy = 0;
  benchmark::Mat<3,3> I_w;
  getRotationMatrix();
  benchmark::similarityTransform(rotMatTemp_, localInertia_, I_w);
  benchmark::vecTransposeMatVecMul(angVelTemp_, I_w, angEnergy);

  // lin
  double linEnergy = 0;
  benchmark::vecDot(linVelTemp_, linVelTemp_, linEnergy);

  return 0.5 * angEnergy + 0.5 * mass_ * linEnergy;
}

double BtMbSingleBodyObject::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
  double potential = 0;
  getBulletPosition();
  benchmark::vecDot(posTemp_, gravity, potential);
  return -potential * mass_;
}

double BtMbSingleBodyObject::getEnergy(const benchmark::Vec<3> &gravity) {
  return getKineticEnergy() + getPotentialEnergy(gravity);
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

void BtMbSingleBodyObject::setBulletPositionAndQuaternion() {
  btVector3 bPosition(
      posTemp_[0],
      posTemp_[1],
      posTemp_[2]
  );
  btQuaternion bQuaternion(
      quatTemp_[1],   // x
      quatTemp_[2],   // y
      quatTemp_[3],   // z
      quatTemp_[0]    // w
  );

  api_->resetBasePositionAndOrientation(objectId_, bPosition, bQuaternion);
}

void BtMbSingleBodyObject::setBulletLinearAndAngularVelocity() {
  btVector3 bLinVel(
      linVelTemp_[0],
      linVelTemp_[1],
      linVelTemp_[2]
  );
  btVector3 bAngVel(
      angVelTemp_[0],  // x
      angVelTemp_[1],  // y
      angVelTemp_[2]   // z
  );

  api_->resetBaseVelocity(objectId_, bLinVel, bAngVel);
}

} // object
} // bullet_mb_sim
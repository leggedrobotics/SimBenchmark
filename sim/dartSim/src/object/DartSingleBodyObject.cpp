//
// Created by kangd on 19.03.18.
//

#include "DartSingleBodyObject.hpp"

namespace dart_sim {
namespace object {

dart_sim::object::DartSingleBodyObject::DartSingleBodyObject(double mass, int id) : mass_(mass), id_(id) {
  skeletonPtr_ = dart::dynamics::Skeleton::create(std::to_string(id) + "_skel");
}

DartSingleBodyObject::~DartSingleBodyObject() {
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> dart_sim::object::DartSingleBodyObject::getQuaternion() {
  updateBodyQuaternion();
  return quat_.e();
}

void dart_sim::object::DartSingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
  updateBodyQuaternion();
  quat = quat_;
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > dart_sim::object::DartSingleBodyObject::getRotationMatrix() {
  updateBodyRotationMatrix();
  return rotMat_.e();
}

void dart_sim::object::DartSingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
  updateBodyRotationMatrix();
  rotation = rotMat_;
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::DartSingleBodyObject::getPosition() {
  updateBodyPosition();
  return pos_.e();
}
void dart_sim::object::DartSingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
  updateBodyPosition();
  pos_w = {pos_[0], pos_[1], pos_[2]};
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::DartSingleBodyObject::getComPosition() {
  updateBodyPosition();
  return pos_.e();
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::DartSingleBodyObject::getLinearVelocity() {
  linVelTemp_[0] = bodyPtr_->getParentJoint()->getVelocity(3);
  linVelTemp_[1] = bodyPtr_->getParentJoint()->getVelocity(4);
  linVelTemp_[2] = bodyPtr_->getParentJoint()->getVelocity(5);
  return linVelTemp_.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::DartSingleBodyObject::getAngularVelocity() {
  angVelTemp_[0] = bodyPtr_->getParentJoint()->getVelocity(0);
  angVelTemp_[1] = bodyPtr_->getParentJoint()->getVelocity(1);
  angVelTemp_[2] = bodyPtr_->getParentJoint()->getVelocity(2);
  return angVelTemp_.e();
}

void dart_sim::object::DartSingleBodyObject::setPosition(Eigen::Vector3d originPosition) {
  setBodyPosition(originPosition);
}

void dart_sim::object::DartSingleBodyObject::setPosition(double x, double y, double z) {
  setBodyPosition({x, y, z});
}

void dart_sim::object::DartSingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {
  setBodyQuaternion(quaternion);
}

void dart_sim::object::DartSingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {
  setBodyRotationMatrix(rotationMatrix);
}

void dart_sim::object::DartSingleBodyObject::setOrientation(double w, double x, double y, double z) {
  setBodyQuaternion({w, x, y, z});
}

void dart_sim::object::DartSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {
  setBodyPosition(originPosition);
  setBodyQuaternion(quaternion);
}

void dart_sim::object::DartSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {
  setBodyPosition(originPosition);
  setBodyRotationMatrix(rotationMatrix);
}

void dart_sim::object::DartSingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {
  Eigen::Vector6d velocity(Eigen::Vector6d::Zero());
  velocity[0] = angularVelocity.x();
  velocity[1] = angularVelocity.y();
  velocity[2] = angularVelocity.z();
  velocity[3] = linearVelocity.x();
  velocity[4] = linearVelocity.y();
  velocity[5] = linearVelocity.z();

  bodyPtr_->getParentJoint()->setVelocities(velocity);
}

void dart_sim::object::DartSingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
  Eigen::Vector6d velocity(Eigen::Vector6d::Zero());
  velocity[0] = wx;
  velocity[1] = wy;
  velocity[2] = wz;
  velocity[3] = dx;
  velocity[4] = dy;
  velocity[5] = dz;

  bodyPtr_->getParentJoint()->setVelocities(velocity);
}

void dart_sim::object::DartSingleBodyObject::setExternalForce(Eigen::Vector3d force) {
  bodyPtr_->setExtForce(force);
}

void dart_sim::object::DartSingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  bodyPtr_->setExtTorque(torque);
}

void dart_sim::object::DartSingleBodyObject::setRestitutionCoefficient(double restitution) {
  bodyPtr_->setRestitutionCoeff(restitution);
}

void dart_sim::object::DartSingleBodyObject::setFrictionCoefficient(double friction) {
  bodyPtr_->setFrictionCoeff(friction);
}

void DartSingleBodyObject::setOrientationRandom() {
  RAIFATAL("not implemented yet")
}

bool dart_sim::object::DartSingleBodyObject::isVisualizeFramesAndCom() const {
  RAIFATAL("not implemented yet")
  return false;
}

void DartSingleBodyObject::setBodyPosition(Eigen::Vector3d pos) {
  pos_ = {pos.x(), pos.y(), pos.z()};
  bodyPtr_->getParentJoint()->setPosition(3, pos.x());
  bodyPtr_->getParentJoint()->setPosition(4, pos.y());
  bodyPtr_->getParentJoint()->setPosition(5, pos.z());
}

void DartSingleBodyObject::setBodyQuaternion(Eigen::Quaterniond quat) {
  quat_ = {quat.w(), quat.x(), quat.y(), quat.z()};

  Eigen::Vector3d rot = dart::math::quatToExp(quat);
  bodyPtr_->getParentJoint()->setPosition(0, rot[0]);
  bodyPtr_->getParentJoint()->setPosition(1, rot[1]);
  bodyPtr_->getParentJoint()->setPosition(2, rot[2]);
}

void DartSingleBodyObject::setBodyRotationMatrix(Eigen::Matrix3d rotationMatrix) {
  rotMat_.e() = rotationMatrix;

  Eigen::Vector3d rot = dart::math::logMap(rotationMatrix);
  bodyPtr_->getParentJoint()->setPosition(0, rot[0]);
  bodyPtr_->getParentJoint()->setPosition(1, rot[1]);
  bodyPtr_->getParentJoint()->setPosition(2, rot[2]);
}

void DartSingleBodyObject::updateBodyPosition() {
  Eigen::Isometry3d tf = bodyPtr_->getParentJoint()->getRelativeTransform();
  pos_ = {tf.translation().x(), tf.translation().y(), tf.translation().z()};
}

void DartSingleBodyObject::updateBodyQuaternion() {
  Eigen::Quaterniond quaternion(bodyPtr_->getParentJoint()->getRelativeTransform().rotation());
  quat_ = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
}

void DartSingleBodyObject::updateBodyRotationMatrix() {
  rotMat_.e() = bodyPtr_->getParentJoint()->getRelativeTransform().linear();
}

} // object
} // dart_sim


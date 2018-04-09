//
// Created by kangd on 19.03.18.
//

#include "SingleBodyObject.hpp"

namespace dart_sim {
namespace object {

dart_sim::object::SingleBodyObject::SingleBodyObject(double mass, int id) : mass_(mass), id_(id) {
  tf_.setIdentity();
}

SingleBodyObject::~SingleBodyObject() {
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> dart_sim::object::SingleBodyObject::getQuaternion() {
  updateTransform();
  Eigen::Quaterniond quaterniond(tf_.rotation());
  quat_ = {quaterniond.w(), quaterniond.x(), quaterniond.y(), quaterniond.z()};
  return quat_.e();
}

void dart_sim::object::SingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
  updateTransform();
  Eigen::Quaterniond quaterniond(tf_.rotation());
  quat_ = {quaterniond.w(), quaterniond.x(), quaterniond.y(), quaterniond.z()};
  quat = quat_;
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > dart_sim::object::SingleBodyObject::getRotationMatrix() {
  updateTransform();
  rotMat_.e() = Eigen::Matrix3d(tf_.rotation());
  return rotMat_.e();
}

void dart_sim::object::SingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
  updateTransform();
  rotMat_.e() = Eigen::Matrix3d(tf_.rotation());
  rotation = rotMat_;
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getPosition() {
  updateTransform();
  pos_.e() = tf_.translation();
  return pos_.e();
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getComPosition() {
  updateTransform();
  pos_.e() = tf_.translation();
  return pos_.e();
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getLinearVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getAngularVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

void dart_sim::object::SingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
  updateTransform();
  pos_.e() = tf_.translation();
  pos_w = {pos_[0], pos_[1], pos_[2]};
}

void dart_sim::object::SingleBodyObject::setPosition(Eigen::Vector3d originPosition) {
  setBodyPosition({originPosition.x(), originPosition.y(), originPosition.z()});
}

void dart_sim::object::SingleBodyObject::setPosition(double x, double y, double z) {
  setBodyPosition({x, y, z});
}

void dart_sim::object::SingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {
  setBodyQuaternion(quaternion);
}

void dart_sim::object::SingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {
  setBodyRotationMatrix(rotationMatrix);
}

void dart_sim::object::SingleBodyObject::setOrientation(double w, double x, double y, double z) {
  Eigen::Quaterniond quaternion;
  quaternion = {w, x, y, z};
  setBodyQuaternion(quaternion);
}

void dart_sim::object::SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {
  RAIFATAL("not implemented yet")
}

void dart_sim::object::SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {
  RAIFATAL("not implemented yet")
}

void dart_sim::object::SingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {
  Eigen::Vector6d velocity(Eigen::Vector6d::Zero());
  velocity[0] = angularVelocity.x();
  velocity[1] = angularVelocity.y();
  velocity[2] = angularVelocity.z();
  velocity[3] = linearVelocity.x();
  velocity[4] = linearVelocity.y();
  velocity[5] = linearVelocity.z();

  bodyPtr_->getParentJoint()->setVelocities(velocity);
}

void dart_sim::object::SingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
  Eigen::Vector6d velocity(Eigen::Vector6d::Zero());
  velocity[0] = wx;
  velocity[1] = wy;
  velocity[2] = wz;
  velocity[3] = dx;
  velocity[4] = dy;
  velocity[5] = dz;

  bodyPtr_->getParentJoint()->setVelocities(velocity);
}

void dart_sim::object::SingleBodyObject::setExternalForce(Eigen::Vector3d force) {
  RAIFATAL("not implemented yet")
}

void dart_sim::object::SingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  RAIFATAL("not implemented yet")
}

void dart_sim::object::SingleBodyObject::setRestitutionCoefficient(double restitution) {
  RAIFATAL("not implemented yet")
}

void dart_sim::object::SingleBodyObject::setFrictionCoefficient(double friction) {
  RAIFATAL("not implemented yet")
}

void SingleBodyObject::setOrientationRandom() {
  RAIFATAL("not implemented yet")
}

bool dart_sim::object::SingleBodyObject::isVisualizeFramesAndCom() const {
  RAIFATAL("not implemented yet")
  return false;
}

void SingleBodyObject::updateTransform() {
  tf_ = bodyPtr_->getParentJoint()->getRelativeTransform();
}

void SingleBodyObject::setBodyPosition(benchmark::Vec<3> position) {
  tf_.translation() = Eigen::Vector3d(position[0], position[1], position[2]);

  dart::dynamics::FreeJoint::setTransform(
      bodyPtr_->getParentJoint(),
      tf_,
      dart::dynamics::Frame::World()
  );
}

void SingleBodyObject::setBodyQuaternion(Eigen::Quaterniond &quaternion) {
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translate(tf_.translation());
  tf.rotate(quaternion);

  dart::dynamics::FreeJoint::setTransform(
      bodyPtr_->getParentJoint(),
      tf,
      dart::dynamics::Frame::World()
  );
}

void SingleBodyObject::setBodyRotationMatrix(Eigen::Matrix3d rotation) {
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = tf_.translation();
  tf.rotate(rotation);

  dart::dynamics::FreeJoint::setTransform(
      bodyPtr_->getParentJoint(),
      tf,
      dart::dynamics::Frame::World()
  );
}

} // object
} // dart_sim


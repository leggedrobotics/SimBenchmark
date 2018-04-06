//
// Created by kangd on 19.03.18.
//

#include "SingleBodyObject.hpp"

namespace dart_sim {
namespace object {

dart_sim::object::SingleBodyObject::SingleBodyObject(double mass): mass_(mass) {
}

SingleBodyObject::~SingleBodyObject() {
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> dart_sim::object::SingleBodyObject::getQuaternion() {
  Eigen::Quaterniond quaternion = getBodyQuaternion();
  benchmark::Vec<4> quat;
  quat = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  return quat.e();
}

void dart_sim::object::SingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
  Eigen::Quaterniond quaternion = getBodyQuaternion();
  quat = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > dart_sim::object::SingleBodyObject::getRotationMatrix() {
  Eigen::Matrix3d rotMat = getBodyRotationMatrix();
  benchmark::Mat<3,3> rotation;
  rotation.e() = rotMat;
  return rotation.e();
}

void dart_sim::object::SingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
  Eigen::Matrix3d rotMat = getBodyRotationMatrix();
  rotation.e() = rotMat;
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getPosition() {
  benchmark::Vec<3> pos;
  pos.e() = getBodyPosition();
  return pos.e();
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getComPosition() {
  benchmark::Vec<3> pos;
  pos.e() = getBodyPosition();
  return pos.e();
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getLinearVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getAngularVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

void dart_sim::object::SingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
  Eigen::Vector3d position = getBodyPosition();
  pos_w = {position[0], position[1], position[2]};
}

void dart_sim::object::SingleBodyObject::setPosition(Eigen::Vector3d originPosition) {
  setBodyPosition(originPosition);
}
void dart_sim::object::SingleBodyObject::setPosition(double x, double y, double z) {
  Eigen::Vector3d positions(x, y, z);
  setBodyPosition(positions);
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

Eigen::Vector3d SingleBodyObject::getBodyPosition() {
  Eigen::Isometry3d tf = bodyPtr_->getParentJoint()->getRelativeTransform();
  return tf.translation();
}

Eigen::Quaterniond SingleBodyObject::getBodyQuaternion() {
  Eigen::Isometry3d tf = bodyPtr_->getParentJoint()->getRelativeTransform();
  return Eigen::Quaterniond(tf.rotation());
}

Eigen::Matrix3d SingleBodyObject::getBodyRotationMatrix() {
  Eigen::Isometry3d tf = bodyPtr_->getParentJoint()->getRelativeTransform();
  return tf.rotation();
}

void SingleBodyObject::setBodyPosition(const Eigen::Vector3d &position) {
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translate(position);

  dart::dynamics::FreeJoint::setTransform(
      bodyPtr_->getParentJoint(),
      tf,
      dart::dynamics::Frame::World()
  );
}
void SingleBodyObject::setBodyQuaternion(Eigen::Quaterniond &quaternion) {
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.rotate(quaternion);

  dart::dynamics::FreeJoint::setTransform(
      bodyPtr_->getParentJoint(),
      tf,
      dart::dynamics::Frame::World()
  );
}

void SingleBodyObject::setBodyRotationMatrix(Eigen::Matrix3d rotation) {
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.rotate(rotation);

  dart::dynamics::FreeJoint::setTransform(
      bodyPtr_->getParentJoint(),
      tf,
      dart::dynamics::Frame::World()
  );
}

} // object
} // dart_sim


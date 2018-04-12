//
// Created by kangd on 19.03.18.
//

#include "DartSingleBodyObject.hpp"

namespace dart_sim {
namespace object {

dart_sim::object::DartSingleBodyObject::DartSingleBodyObject(double mass, int id) : mass_(mass), id_(id) {
}

DartSingleBodyObject::~DartSingleBodyObject() {
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> dart_sim::object::DartSingleBodyObject::getQuaternion() {
  dartTf2States();
  return quat_.e();
}

void dart_sim::object::DartSingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
  dartTf2States();
  quat = quat_;
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > dart_sim::object::DartSingleBodyObject::getRotationMatrix() {
  dartTf2States();
  return rotMat_.e();
}

void dart_sim::object::DartSingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
  dartTf2States();
  rotation = rotMat_;
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::DartSingleBodyObject::getPosition() {
  dartTf2States();
  return pos_.e();
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::DartSingleBodyObject::getComPosition() {
  dartTf2States();
  return pos_.e();
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::DartSingleBodyObject::getLinearVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::DartSingleBodyObject::getAngularVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

void dart_sim::object::DartSingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
  dartTf2States();
  pos_w = {pos_[0], pos_[1], pos_[2]};
}

void dart_sim::object::DartSingleBodyObject::setPosition(Eigen::Vector3d originPosition) {
  pos_ = {originPosition[0], originPosition[1], originPosition[2]};
  states2DartTf();
}

void dart_sim::object::DartSingleBodyObject::setPosition(double x, double y, double z) {
  pos_ = {x, y, z};
  states2DartTf();
}

void dart_sim::object::DartSingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {
  quat_ = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  states2DartTf();
}

void dart_sim::object::DartSingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {
  Eigen::Quaterniond quaternion(rotationMatrix);
  quat_ = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  states2DartTf();
}

void dart_sim::object::DartSingleBodyObject::setOrientation(double w, double x, double y, double z) {
  Eigen::Quaterniond quaternion = {w, x, y, z};
  quat_ = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  states2DartTf();
}

void dart_sim::object::DartSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {
  RAIFATAL("not implemented yet")
}

void dart_sim::object::DartSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {
  RAIFATAL("not implemented yet")
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
  RAIFATAL("not implemented yet")
}

void dart_sim::object::DartSingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  RAIFATAL("not implemented yet")
}

void dart_sim::object::DartSingleBodyObject::setRestitutionCoefficient(double restitution) {
  RAIFATAL("not implemented yet")
}

void dart_sim::object::DartSingleBodyObject::setFrictionCoefficient(double friction) {
  RAIFATAL("not implemented yet")
}

void DartSingleBodyObject::setOrientationRandom() {
  RAIFATAL("not implemented yet")
}

bool dart_sim::object::DartSingleBodyObject::isVisualizeFramesAndCom() const {
  RAIFATAL("not implemented yet")
  return false;
}

void DartSingleBodyObject::dartTf2States() {
  Eigen::Isometry3d tf = bodyPtr_->getParentJoint()->getRelativeTransform();
  pos_ = {tf.translation().x(), tf.translation().y(), tf.translation().z()};
  Eigen::Quaterniond quaternion(tf.rotation());
  quat_ = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  Eigen::Matrix3d rotationMatrix(tf.rotation());
  rotMat_.e() = rotationMatrix;
}

void DartSingleBodyObject::states2DartTf() {
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation()[0] = pos_[0];
  tf.translation()[1] = pos_[1];
  tf.translation()[2] = pos_[2];

  Eigen::Quaterniond quaternion(quat_[0], quat_[1], quat_[2], quat_[3]);
  tf.rotate(quaternion);

  dart::dynamics::FreeJoint::setTransform(
      bodyPtr_->getParentJoint(),
      tf,
      dart::dynamics::Frame::World()
  );
}

} // object
} // dart_sim


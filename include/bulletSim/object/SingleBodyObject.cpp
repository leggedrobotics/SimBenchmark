//
// Created by kangd on 10.02.18.
//

#include "SingleBodyObject.hpp"

bullet_sim::object::SingleBodyObject::SingleBodyObject(double mass) : mass_(mass) {}

bullet_sim::object::SingleBodyObject::~SingleBodyObject() {
  delete rigidBody_;
  delete motionState_;
  delete collisionShape_;
}


bool bullet_sim::object::SingleBodyObject::isVisualizeFramesAndCom() const {
  return visualizeFramesAndCom_;
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> bullet_sim::object::SingleBodyObject::getQuaternion() {
  const btQuaternion &quaternion = rigidBody_->getWorldTransform().getRotation();
  rai_sim::Vec<4> quat = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  return quat.e();
}

void bullet_sim::object::SingleBodyObject::getQuaternion(rai_sim::Vec<4> &quat) {
  const btQuaternion &quaternion = rigidBody_->getWorldTransform().getRotation();
  quat = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
}

void bullet_sim::object::SingleBodyObject::getRotationMatrix(rai_sim::Mat<3, 3> &rotation) {
  const btQuaternion &quaternion = rigidBody_->getWorldTransform().getRotation();
  RAIFATAL('not implemented yet');
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > bullet_sim::object::SingleBodyObject::getRotationMatrix() {
  RAIFATAL('not implemented yet');
  return Eigen::Map<Eigen::Matrix<double, 3, 3>>(nullptr);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > bullet_sim::object::SingleBodyObject::getPosition() {
  const btVector3 &position = rigidBody_->getWorldTransform().getOrigin();
  rai_sim::Vec<3> pos = {position.x(), position.y(), position.z()};
  return pos.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > bullet_sim::object::SingleBodyObject::getComPosition() {
  const btVector3 &position = rigidBody_->getWorldTransform().getOrigin();
  rai_sim::Vec<3> pos = {position.x(), position.y(), position.z()};
  RAIWARN('check if COM = body origin!');
  return pos.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > bullet_sim::object::SingleBodyObject::getLinearVelocity() {
  const btVector3 &linearVelocity = rigidBody_->getLinearVelocity();
  rai_sim::Vec<3> linvel = {linearVelocity.x(), linearVelocity.y(), linearVelocity.z()};
  return linvel.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > bullet_sim::object::SingleBodyObject::getAngularVelocity() {
  const btVector3 &angularVelocity = rigidBody_->getAngularVelocity();
  rai_sim::Vec<3> angvel = {angularVelocity.x(), angularVelocity.y(), angularVelocity.z()};
  return angvel.e();
}

void bullet_sim::object::SingleBodyObject::getPosition_W(rai_sim::Vec<3> &pos_w) {
  const btVector3 &position = rigidBody_->getWorldTransform().getOrigin();
  pos_w = {position.getX(), position.getY(), position.getZ()};
}

void bullet_sim::object::SingleBodyObject::setPosition(Eigen::Vector3d originPosition) {
  rigidBody_->getWorldTransform().setOrigin(btVector3(originPosition[0],
                                                      originPosition[1],
                                                      originPosition[2]));
}

void bullet_sim::object::SingleBodyObject::setPosition(double x, double y, double z) {
  rigidBody_->getWorldTransform().setOrigin(btVector3(x, y, z));
}

void bullet_sim::object::SingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {
  setOrientation(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
}

void bullet_sim::object::SingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {
  Eigen::Quaterniond quaternion(rotationMatrix);
  setOrientation(quaternion);
}

void bullet_sim::object::SingleBodyObject::setOrientation(double w, double x, double y, double z) {
  rigidBody_->getWorldTransform().setRotation(btQuaternion(x, y, z, w));
}

void bullet_sim::object::SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {
  setPosition(originPosition);
  setOrientation(quaternion);
}

void bullet_sim::object::SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {
  setPosition(originPosition);
  setOrientation(rotationMatrix);
}

void bullet_sim::object::SingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity,
                                                       Eigen::Vector3d angularVelocity) {
  setVelocity(linearVelocity[0], linearVelocity[1], linearVelocity[2],
              angularVelocity[0], angularVelocity[1], angularVelocity[2]);
}

void bullet_sim::object::SingleBodyObject::setVelocity(double dx,
                                                       double dy,
                                                       double dz,
                                                       double wx,
                                                       double wy,
                                                       double wz) {
  rigidBody_->setLinearVelocity(btVector3(dx, dy, dz));
  rigidBody_->setAngularVelocity(btVector3(wx, wy, wz));
}

btRigidBody *bullet_sim::object::SingleBodyObject::getRigidBody() const {
  return rigidBody_;
}
void bullet_sim::object::SingleBodyObject::setRestitution(double restitution) {
  rigidBody_->setRestitution(restitution);
}
void bullet_sim::object::SingleBodyObject::setFriction(double friction) {
  rigidBody_->setFriction(friction);
}

void bullet_sim::object::SingleBodyObject::setExternalForce(Eigen::Vector3d force) {
  rigidBody_->applyCentralForce(btVector3(force[0], force[1], force[2]));
}

void bullet_sim::object::SingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  rigidBody_->applyTorque(btVector3(torque[0], torque[1], torque[2]));
}

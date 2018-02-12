//
// Created by kangd on 10.02.18.
//

#include "bulletSim/object/SingleBodyObject.hpp"

bullet_sim::object::SingleBodyObject::SingleBodyObject(double mass) : mass_(mass) {}

bullet_sim::object::SingleBodyObject::~SingleBodyObject() {
  delete motionState_;
  delete rigidBody_;
  delete collisionShape_;
}


bool bullet_sim::object::SingleBodyObject::isVisualizeFramesAndCom() const {
  return visualizeFramesAndCom_;
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> bullet_sim::object::SingleBodyObject::getQuaternion() {
  return Eigen::Map<Eigen::Matrix<double, 4, 1>>(nullptr);
}

void bullet_sim::object::SingleBodyObject::getQuaternion(rai_sim::Vec<4> &quat) {
}

void bullet_sim::object::SingleBodyObject::getRotationMatrix(rai_sim::Mat<3, 3> &rotation) {
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > bullet_sim::object::SingleBodyObject::getRotationMatrix() {
  return Eigen::Map<Eigen::Matrix<double, 3, 3>>(nullptr);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > bullet_sim::object::SingleBodyObject::getPosition() {
  btVector3 position = rigidBody_->getWorldTransform().getOrigin();
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > bullet_sim::object::SingleBodyObject::getComPosition() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > bullet_sim::object::SingleBodyObject::getLinearVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > bullet_sim::object::SingleBodyObject::getAngularVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
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
// TODO
  assert('not implemented yet');
}

btRigidBody *bullet_sim::object::SingleBodyObject::getRigidBody_() const {
  return rigidBody_;
}

//
// Created by kangd on 11.02.18.
//

#include "odeSim/object/SingleBodyObject.hpp"

namespace ode_sim {

object::SingleBodyObject::SingleBodyObject(const dWorldID worldID) : worldID_(worldID) {}

object::SingleBodyObject::~SingleBodyObject() {
  dBodyDestroy(body_);
  dGeomDestroy(geometry_);
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> ode_sim::object::SingleBodyObject::getQuaternion() {
  return Eigen::Map<Eigen::Matrix<double, 4, 1>>(nullptr);
}

void ode_sim::object::SingleBodyObject::getQuaternion(rai_sim::Vec<4> &quat) {

}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > ode_sim::object::SingleBodyObject::getRotationMatrix() {
  return Eigen::Map<Eigen::Matrix<double, 3, 3>>(nullptr);
}

void ode_sim::object::SingleBodyObject::getRotationMatrix(rai_sim::Mat<3, 3> &rotation) {

}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::SingleBodyObject::getPosition() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::SingleBodyObject::getComPosition() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::SingleBodyObject::getLinearVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::SingleBodyObject::getAngularVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

void ode_sim::object::SingleBodyObject::getPosition_W(rai_sim::Vec<3> &pos_w) {
  const dReal *position = dGeomGetPosition(geometry_);
  pos_w = {position[0], position[1], position[2]};
}

void ode_sim::object::SingleBodyObject::setPosition(Eigen::Vector3d originPosition) {
  dGeomSetPosition(geometry_, originPosition[0], originPosition[1], originPosition[2]);
}

void ode_sim::object::SingleBodyObject::setPosition(double x, double y, double z) {
  dGeomSetPosition(geometry_, x, y, z);
}

void ode_sim::object::SingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {
  dQuaternion dquaternion = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  dGeomSetQuaternion(geometry_, dquaternion);
}

void ode_sim::object::SingleBodyObject::setOrientation(double w, double x, double y, double z) {
  dQuaternion dquaternion = {w, x, y, z};
  dGeomSetQuaternion(geometry_, dquaternion);
}

void ode_sim::object::SingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {
  dMatrix3 drotation;
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      drotation[4*i + j] = rotationMatrix(i, j);
    }
    drotation[4*i + 3] = 0;
  }
  dBodySetRotation(body_, drotation);
}

void ode_sim::object::SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {
  setPosition(originPosition);
  setOrientation(quaternion);
}

void ode_sim::object::SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {
  setPosition(originPosition);
  setOrientation(rotationMatrix);
}

void ode_sim::object::SingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {
  setVelocity(linearVelocity[0], linearVelocity[1], linearVelocity[2],
              angularVelocity[0], angularVelocity[1], angularVelocity[2]);
}

void ode_sim::object::SingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
// TODO
  assert('not implemented yet');
}

bool ode_sim::object::SingleBodyObject::isVisualizeFramesAndCom() const {
  return visualizeFramesAndCom_;
}

} // ode_sim

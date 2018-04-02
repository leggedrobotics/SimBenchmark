//
// Created by kangd on 18.02.18.
//

#include "SingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

SingleBodyObject::SingleBodyObject(mjData *data,
                                   mjModel *model,
                                   int bodyId,
                                   int geomId)
    : worldData_(data), worldModel_(model), bodyID_(bodyId), geomID_(geomId) {}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> SingleBodyObject::getQuaternion() {
  mjtNum *rotMat = getGeomRotMat();
  rotMatTemp_.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8];
  benchmark::rotMatToQuat(rotMatTemp_, quatTemp_);
  return quatTemp_.e();
}

void SingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
  mjtNum *rotMat = getGeomRotMat();
  rotMatTemp_.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8];
  benchmark::rotMatToQuat(rotMatTemp_, quat);
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > SingleBodyObject::getRotationMatrix() {
  mjtNum *rotMat = getGeomRotMat();
  rotMatTemp_.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8];
  return rotMatTemp_.e();
}

void SingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
  mjtNum *rotMat = getGeomRotMat();
  rotation.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8];
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > SingleBodyObject::getPosition() {
  mjtNum *pos = getGeomPosition();
  posTemp_ = {pos[0], pos[1], pos[2]};
  return posTemp_.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > SingleBodyObject::getComPosition() {
  mjtNum *pos = getBodyComPosition();
  posTemp_ = {pos[0], pos[1], pos[2]};
  return posTemp_.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > SingleBodyObject::getLinearVelocity() {
  mjtNum *linVel = getBodyLinearVelocity();
  linVelTemp_ = {linVel[0], linVel[1], linVel[2]};
  return linVelTemp_.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > SingleBodyObject::getAngularVelocity() {
  mjtNum *angVel = getBodyAngularVelocity();
  angVelTemp_ = {angVel[0], angVel[1], angVel[2]};
  return angVelTemp_.e();
}

void SingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
  mjtNum *pos = getGeomPosition();
  pos_w = {pos[0], pos[1], pos[2]};
}

bool SingleBodyObject::isVisualizeFramesAndCom() const {
  return visualizeFramesAndCom_;
}

void SingleBodyObject::setExternalForce(Eigen::Vector3d force) {
  mjtNum *extforce = (worldData_->xfrc_applied + 6 * bodyID_);
  extforce[0] = force[0];
  extforce[1] = force[1];
  extforce[2] = force[2];
}
void SingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  mjtNum *extforce = (worldData_->xfrc_applied + 6 * bodyID_ + 3);
  extforce[0] = torque[0];
  extforce[1] = torque[1];
  extforce[2] = torque[2];
}
mjtNum *SingleBodyObject::getGeomPosition() {
  int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_;
  return worldData_->geom_xpos + 3 * geomIndex;
}

mjtNum *SingleBodyObject::getGeomRotMat() {
  int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_;
  return worldData_->geom_xmat + 9 * geomIndex;
}

mjtNum *SingleBodyObject::getBodyComPosition() {
  return worldData_->xipos + 3 * bodyID_;
}

mjtNum *SingleBodyObject::getBodyLinearVelocity() {
  return worldData_->cvel + 6 * bodyID_ + 3;
}

mjtNum *SingleBodyObject::getBodyAngularVelocity() {
  return worldData_->cvel + 6 * bodyID_;
}

/// deprecated
/// ===================================================================
void SingleBodyObject::setPosition(Eigen::Vector3d originPosition) {

}

void SingleBodyObject::setPosition(double x, double y, double z) {

}
void SingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {

}
void SingleBodyObject::setOrientation(double w, double x, double y, double z) {

}
void SingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {

}
void SingleBodyObject::setOrientationRandom() {

}
void SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {

}
void SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {

}
void SingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {

}
void SingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {

}
void SingleBodyObject::setRestitutionCoefficient(double restitution) {

}

void SingleBodyObject::setFrictionCoefficient(double friction) {

}

} // object
} // mujoco_sim
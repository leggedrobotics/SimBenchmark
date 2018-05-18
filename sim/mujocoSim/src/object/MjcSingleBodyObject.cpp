//
// Created by kangd on 18.02.18.
//

#include "MjcSingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

MjcSingleBodyObject::MjcSingleBodyObject(mjData *data,
                                   mjModel *model,
                                   int bodyId,
                                   int geomId)
    : worldData_(data), worldModel_(model), bodyID_(bodyId), geomID_(geomId) {

  // init physical properties
  setGeomFriction({0.8, 0, 0});
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> MjcSingleBodyObject::getQuaternion() {
  mjtNum *rotMat = getGeomRotMat();
  rotMatTemp_.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8];
  benchmark::rotMatToQuat(rotMatTemp_, quatTemp_);
  return quatTemp_.e();
}

void MjcSingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
  mjtNum *rotMat = getGeomRotMat();
  rotMatTemp_.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8];
  benchmark::rotMatToQuat(rotMatTemp_, quat);
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > MjcSingleBodyObject::getRotationMatrix() {
  mjtNum *rotMat = getGeomRotMat();
  rotMatTemp_.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8];
  return rotMatTemp_.e();
}

void MjcSingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
  mjtNum *rotMat = getGeomRotMat();
  rotation.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8];
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > MjcSingleBodyObject::getPosition() {
  mjtNum *pos = getGeomPosition();
  posTemp_ = {pos[0], pos[1], pos[2]};
  return posTemp_.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > MjcSingleBodyObject::getComPosition() {
  mjtNum *pos = getBodyComPosition();
  posTemp_ = {pos[0], pos[1], pos[2]};
  return posTemp_.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > MjcSingleBodyObject::getLinearVelocity() {
  mjtNum *linVel = getBodyLinearVelocity();
  linVelTemp_ = {linVel[0], linVel[1], linVel[2]};
  return linVelTemp_.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > MjcSingleBodyObject::getAngularVelocity() {
  mjtNum *angVel = getBodyAngularVelocity();
  angVelTemp_ = {angVel[0], angVel[1], angVel[2]};
  return angVelTemp_.e();
}

void MjcSingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
  mjtNum *pos = getGeomPosition();
  pos_w = {pos[0], pos[1], pos[2]};
}

bool MjcSingleBodyObject::isVisualizeFramesAndCom() const {
  return visualizeFramesAndCom_;
}

void MjcSingleBodyObject::setExternalForce(Eigen::Vector3d force) {
  mjtNum *extforce = (worldData_->xfrc_applied + 6 * bodyID_);
  extforce[0] = force[0];
  extforce[1] = force[1];
  extforce[2] = force[2];
}
void MjcSingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  mjtNum *extforce = (worldData_->xfrc_applied + 6 * bodyID_ + 3);
  extforce[0] = torque[0];
  extforce[1] = torque[1];
  extforce[2] = torque[2];
}
mjtNum *MjcSingleBodyObject::getGeomPosition() {
  int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_;
  return worldData_->geom_xpos + 3 * geomIndex;
}

mjtNum *MjcSingleBodyObject::getGeomRotMat() {
  int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_;
  return worldData_->geom_xmat + 9 * geomIndex;
}

mjtNum *MjcSingleBodyObject::getBodyComPosition() {
  return worldData_->xipos + 3 * bodyID_;
}

mjtNum *MjcSingleBodyObject::getBodyLinearVelocity() {
  return worldData_->cvel + 6 * bodyID_ + 3;
}

mjtNum *MjcSingleBodyObject::getBodyAngularVelocity() {
  return worldData_->cvel + 6 * bodyID_;
}

/// friction for (slide, spin, roll)
void MjcSingleBodyObject::setGeomFriction(benchmark::Vec<3> friction) {
  int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_;
  worldModel_->geom_friction[3 * geomIndex] = friction[0];
  worldModel_->geom_friction[3 * geomIndex + 1] = friction[1];
  worldModel_->geom_friction[3 * geomIndex + 2] = friction[2];
}

void MjcSingleBodyObject::setFrictionCoefficient(double friction) {
  setGeomFriction({friction, 0, 0});
}

mjtNum MjcSingleBodyObject::getBodyMass() {
  return worldModel_->body_mass[bodyID_];
}
mjtNum *MjcSingleBodyObject::getBodyInertia() {
  return worldModel_->body_inertia + 3 * bodyID_;
}

double MjcSingleBodyObject::getKineticEnergy() {
  double mass = getBodyMass();
  double *inertia = getBodyInertia();

  getLinearVelocity();
  getAngularVelocity();
  benchmark::Mat<3,3> I;
  I.e() << inertia[0], 0, 0,
      0, inertia[1], 0,
      0, 0, inertia[2];

  // ang
  double angEnergy = 0;
  benchmark::Mat<3,3> I_w;
  getRotationMatrix();
  benchmark::similarityTransform(rotMatTemp_, I, I_w);
  benchmark::vecTransposeMatVecMul(angVelTemp_, I_w, angEnergy);

  // lin
  double linEnergy = 0;
  benchmark::vecDot(linVelTemp_, linVelTemp_, linEnergy);

  return 0.5 * angEnergy + 0.5 * mass * linEnergy;
}

double MjcSingleBodyObject::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
  double potential = 0;
  double mass = getBodyMass();

  getPosition();
  benchmark::vecDot(posTemp_, gravity, potential);
  return -potential * mass;
}

double MjcSingleBodyObject::getEnergy(const benchmark::Vec<3> &gravity) {
  return getKineticEnergy() + getPotentialEnergy(gravity);
}

/// deprecated
/// ===================================================================
void MjcSingleBodyObject::setPosition(Eigen::Vector3d originPosition) {

}
void MjcSingleBodyObject::setPosition(double x, double y, double z) {

}
void MjcSingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {

}
void MjcSingleBodyObject::setOrientation(double w, double x, double y, double z) {

}
void MjcSingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {

}
void MjcSingleBodyObject::setOrientationRandom() {

}
void MjcSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {

}
void MjcSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {

}

void MjcSingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {

}
void MjcSingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {

}
void MjcSingleBodyObject::setRestitutionCoefficient(double restitution) {

}
double MjcSingleBodyObject::getMass() {
  return getBodyMass();
}
bool MjcSingleBodyObject::isMovable() const {
  return isMovable_;
}
const Eigen::Map<Eigen::Matrix<double, 3, 1>> MjcSingleBodyObject::getLinearMomentum() {
  getLinearVelocity();
  double mass = getMass();
  benchmark::vecScalarMul(mass, linVelTemp_, linearMomentum_);
  return linearMomentum_.e();
}

void MjcSingleBodyObject::setCollisionGroupAndMask(int collisionGroup, int collisionMask) {
  int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_;
  worldModel_->geom_contype[geomIndex] = collisionGroup;
  worldModel_->geom_conaffinity[geomIndex] = collisionMask;
}

} // object
} // mujoco_sim
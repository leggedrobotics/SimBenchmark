//
// Created by kangd on 11.02.18.
//

#include "odeSim/object/SingleBodyObject.hpp"

namespace ode_sim {

object::SingleBodyObject::SingleBodyObject(const dWorldID worldID, const dSpaceID spaceID)
    : worldID_(worldID), spaceID_ (spaceID) {}

object::SingleBodyObject::~SingleBodyObject() {
  if(body_)
    dBodyDestroy(body_);
  if(geometry_)
    dGeomDestroy(geometry_);
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> ode_sim::object::SingleBodyObject::getQuaternion() {
//  const dQuaternion *quaternion = dGeomGetQuaternion(geometry_);
  RAIFATAL('not implemented yet');
  return Eigen::Map<Eigen::Matrix<double, 4, 1>>(nullptr);
}

void ode_sim::object::SingleBodyObject::getQuaternion(rai_sim::Vec<4> &quat) {
//  dGeomGetQuaternion(geometry_, &dquaternion);
//  RAIFATAL('not implemented yet');
//  quat = {dquaternion[0], dquaternion[1], dquaternion[2] dquaternion[3]};
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > ode_sim::object::SingleBodyObject::getRotationMatrix() {
  const dReal* rot = dGeomGetRotation(geometry_);
  RAIFATAL('not implemented yet');
  return Eigen::Map<Eigen::Matrix<double, 3, 3>>(nullptr);
}

void ode_sim::object::SingleBodyObject::getRotationMatrix(rai_sim::Mat<3, 3> &rotation) {
  RAIFATAL('not implemented yet');
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::SingleBodyObject::getPosition() {
  const dReal *position = dGeomGetPosition(geometry_);
  rai_sim::Vec<3> pos = {position[0], position[1], position[2]};
  return pos.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::SingleBodyObject::getComPosition() {
  const dReal *position = dGeomGetPosition(geometry_);
  rai_sim::Vec<3> pos = {position[0], position[1], position[2]};
  RAIWARN('check if COM = body origin!');
  return pos.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::SingleBodyObject::getLinearVelocity() {
  const dReal *linearVelocity;
  if(body_) {
    linearVelocity = dBodyGetLinearVel(body_);
  }
  else {
    RAIFATAL('cannot get velocity from static object');
  }
  rai_sim::Vec<3> linvel = {linearVelocity[0], linearVelocity[1], linearVelocity[2]};
  return linvel.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::SingleBodyObject::getAngularVelocity() {
  const dReal *angularVelocity;
  if(body_) {
    angularVelocity = dBodyGetAngularVel(body_);
  }
  else {
    RAIFATAL('cannot get velocity from static object');
  }
  rai_sim::Vec<3> angvel = {angularVelocity[0], angularVelocity[1], angularVelocity[2]};
  return angvel.e();
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
  dGeomSetRotation(geometry_, drotation);
}

void object::SingleBodyObject::setOrientationRandom() {
  Eigen::Vector4d quat(rn_.sampleUniform(), rn_.sampleUniform(), rn_.sampleUniform(), rn_.sampleUniform());
  quat /= quat.norm();
  setOrientation(quat(0), quat(1), quat(2), quat(3));
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
  if(body_) {
    dBodySetLinearVel(body_, dx, dy, dz);
    dBodySetAngularVel(body_, wx, wy, wz);
  }
  else {
    RAIFATAL('cannot set velocity to static object');
  }
}

void object::SingleBodyObject::setExternalForce(Eigen::Vector3d force) {
  if(body_) {
    dBodySetForce(body_, force[0], force[1], force[2]);
  }
  else {
    RAIFATAL('cannot set velocity to static object');
  }
}

void object::SingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  if(body_) {
    dBodySetForce(body_, torque[0], torque[2], torque[2]);
  }
  else {
    RAIFATAL('cannot set torque to static object');
  }
}
bool ode_sim::object::SingleBodyObject::isVisualizeFramesAndCom() const {
  return visualizeFramesAndCom_;
}

} // ode_sim

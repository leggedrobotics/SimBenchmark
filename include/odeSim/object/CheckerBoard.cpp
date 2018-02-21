//
// Created by kangd on 12.02.18.
//

#include <Configure.hpp>
#include "odeSim/object/CheckerBoard.hpp"
#include "SingleBodyObject.hpp"

ode_sim::object::CheckerBoard::CheckerBoard(dWorldID worldId,
                                            dSpaceID spaceID,
                                            benchmark::CollisionGroupType collisionGroup,
                                            benchmark::CollisionGroupType collisionMask)
    : SingleBodyObject(worldId, spaceID) {

  // geometry
  geometry_ = dCreatePlane(spaceID, 0, 0, 1, 0);
  body_ = 0;

  // material prop
  dGeomSetData(geometry_, &matrialProp_);

  // collision group
  dGeomSetCategoryBits(geometry_, collisionGroup);
  dGeomSetCollideBits(geometry_, collisionMask);
}

ode_sim::object::CheckerBoard::~CheckerBoard() {
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> ode_sim::object::CheckerBoard::getQuaternion() {
  RAIFATAL("CANNOT GET POSE OF STATIC PLANE");
  return SingleBodyObject::getQuaternion();
}

void ode_sim::object::CheckerBoard::getQuaternion(rai_sim::Vec<4> &quat) {
  quat = {0, 0, 0, 1};
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > ode_sim::object::CheckerBoard::getRotationMatrix() {
  RAIFATAL("CANNOT GET POSE OF STATIC PLANE");
  return SingleBodyObject::getRotationMatrix();
}

void ode_sim::object::CheckerBoard::getRotationMatrix(rai_sim::Mat<3, 3> &rotation) {
  RAIFATAL("CANNOT GET POSE OF STATIC PLANE");
  SingleBodyObject::getRotationMatrix(rotation);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::CheckerBoard::getPosition() {
  RAIFATAL("CANNOT GET POSE OF STATIC PLANE");
  return SingleBodyObject::getPosition();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::CheckerBoard::getComPosition() {
  RAIFATAL("CANNOT GET POSE OF STATIC PLANE");
  return SingleBodyObject::getComPosition();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::CheckerBoard::getLinearVelocity() {
  RAIFATAL("CANNOT GET VELOCITY OF STATIC PLANE");
  return SingleBodyObject::getLinearVelocity();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::CheckerBoard::getAngularVelocity() {
  RAIFATAL("CANNOT GET VELOCITY OF STATIC PLANE");
  return SingleBodyObject::getAngularVelocity();
}

void ode_sim::object::CheckerBoard::getPosition_W(rai_sim::Vec<3> &pos_w) {
  dVector4 planeParams;
  dGeomPlaneGetParams(geometry_, planeParams);
  pos_w = {0, 0, planeParams[3]};
}
void ode_sim::object::CheckerBoard::setPosition(Eigen::Vector3d originPosition) {
  RAIFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::CheckerBoard::setPosition(double x, double y, double z) {
  RAIFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::CheckerBoard::setOrientation(Eigen::Quaterniond quaternion) {
  RAIFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::CheckerBoard::setOrientation(double w, double x, double y, double z) {
  RAIFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::CheckerBoard::setOrientation(Eigen::Matrix3d rotationMatrix) {
  RAIFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::CheckerBoard::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {
  RAIFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::CheckerBoard::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
  RAIFATAL("CANNOT SET POSE OF STATIC PLANE");
}


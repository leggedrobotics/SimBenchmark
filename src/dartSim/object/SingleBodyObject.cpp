//

#include "SingleBodyObject.hpp"
//
// Created by kangd on 19.03.18.

namespace dart_sim {
namespace object {

dart_sim::object::SingleBodyObject::SingleBodyObject(double mass): mass_(mass) {
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> dart_sim::object::SingleBodyObject::getQuaternion() {
  return Eigen::Map<Eigen::Matrix<double, 4, 1>>(nullptr);
}
void dart_sim::object::SingleBodyObject::getQuaternion(rai_sim::Vec<4> &quat) {

}
const Eigen::Map<Eigen::Matrix<double, 3, 3> > dart_sim::object::SingleBodyObject::getRotationMatrix() {
  return Eigen::Map<Eigen::Matrix<double, 3, 3>>(nullptr);
}
void dart_sim::object::SingleBodyObject::getRotationMatrix(rai_sim::Mat<3, 3> &rotation) {

}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getPosition() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getComPosition() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getLinearVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}
const Eigen::Map<Eigen::Matrix<double, 3, 1> > dart_sim::object::SingleBodyObject::getAngularVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}
void dart_sim::object::SingleBodyObject::getPosition_W(rai_sim::Vec<3> &pos_w) {

}
void dart_sim::object::SingleBodyObject::setPosition(Eigen::Vector3d originPosition) {
  Eigen::Vector6d positions(Eigen::Vector6d::Zero());
  positions[3] = originPosition.x();
  positions[4] = originPosition.y();
  positions[5] = originPosition.z();

  skeletonPtr_->getJoint(0)->setPosition(positions);
}
void dart_sim::object::SingleBodyObject::setPosition(double x, double y, double z) {
  Eigen::Vector6d positions(Eigen::Vector6d::Zero());
  positions[3] = x;
  positions[4] = y;
  positions[5] = z;

  skeletonPtr_->getJoint(0)->setPosition(positions);
}
void dart_sim::object::SingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {

}
void dart_sim::object::SingleBodyObject::setOrientation(double w, double x, double y, double z) {

}
void dart_sim::object::SingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {

}
void dart_sim::object::SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {

}
void dart_sim::object::SingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {

}
void dart_sim::object::SingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {

}
void dart_sim::object::SingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {

}
void dart_sim::object::SingleBodyObject::setExternalForce(Eigen::Vector3d force) {

}
void dart_sim::object::SingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {

}
void dart_sim::object::SingleBodyObject::setRestitutionCoefficient(double restitution) {

}
void dart_sim::object::SingleBodyObject::setFrictionCoefficient(double friction) {

}
bool dart_sim::object::SingleBodyObject::isVisualizeFramesAndCom() const {
  return false;
}
const dart::dynamics::SkeletonPtr &SingleBodyObject::getSkeletonPtr() const {
  return skeletonPtr_;
}

} // object
} // dart_sim


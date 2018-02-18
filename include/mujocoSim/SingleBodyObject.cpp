//
// Created by kangd on 18.02.18.
//

#include "SingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

SingleBodyObject::SingleBodyObject(mjData *data, int objectID) : worldData_(data), objectID_(objectID) {}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> SingleBodyObject::getQuaternion() {
  return Eigen::Map<Eigen::Matrix<double, 4, 1>>(nullptr);
}

void SingleBodyObject::getQuaternion(rai_sim::Vec<4> &quat) {

}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > SingleBodyObject::getRotationMatrix() {
  return Eigen::Map<Eigen::Matrix<double, 3, 3>>(nullptr);
}

void SingleBodyObject::getRotationMatrix(rai_sim::Mat<3, 3> &rotation) {

}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > SingleBodyObject::getPosition() {
  mjtNum *pos = (worldData_->xpos + 3 * objectID_);
  posTemp_ = {pos[0], pos[1], pos[2]};
  return posTemp_.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > SingleBodyObject::getComPosition() {
  mjtNum *pos = (worldData_->xpos + 3 * objectID_);
  posTemp_ = {pos[0], pos[1], pos[2]};
  RAIWARN('check if COM = body origin!');
  return posTemp_.e();
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > SingleBodyObject::getLinearVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > SingleBodyObject::getAngularVelocity() {
  return Eigen::Map<Eigen::Matrix<double, 3, 1>>(nullptr);
}
void SingleBodyObject::getPosition_W(rai_sim::Vec<3> &pos_w) {

}

} // object
} // mujoco_sim
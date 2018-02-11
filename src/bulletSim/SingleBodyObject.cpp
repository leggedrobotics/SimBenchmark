//
// Created by kangd on 10.02.18.
//

#include "bulletSim/object/SingleBodyObject.hpp"

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
  pos_w = {
      rigidBody_->getWorldTransform().getOrigin().getX(),
      rigidBody_->getWorldTransform().getOrigin().getY(),
      rigidBody_->getWorldTransform().getOrigin().getZ()
  };
}

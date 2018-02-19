//
// Created by kangd on 18.02.18.
//

#ifndef MUJOCOSIM_SINGLEBODYOBJECT_HPP
#define MUJOCOSIM_SINGLEBODYOBJECT_HPP

#include <Eigen/Geometry>
#include <raiSim/math.hpp>
#include <mujoco.h>

namespace mujoco_sim {
namespace object {

class SingleBodyObject {

 public:
  SingleBodyObject(mjData *data, int objectID);

  virtual const Eigen::Map<Eigen::Matrix<double, 4, 1>> getQuaternion();
  virtual void getQuaternion(rai_sim::Vec<4>& quat);
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 3> > getRotationMatrix();
  virtual void getRotationMatrix(rai_sim::Mat<3,3>& rotation);
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getPosition();
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getComPosition();
  virtual  const Eigen::Map<Eigen::Matrix<double, 3, 1> > getLinearVelocity();
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getAngularVelocity();
  virtual void getPosition_W(rai_sim::Vec<3>& pos_w);

 protected:
  int objectID_ = 0;

  mjData* worldData_;

  // pose and velocity
  rai_sim::Vec<4> quatTemp_ = {1.0, 0.0, 0.0, 0.0};
  rai_sim::Mat<3, 3> rotMatTemp_;
  rai_sim::Vec<3> posTemp_ = {0.0, 0.0, 0.0};
  rai_sim::Vec<3> linVelTemp_ = {0.0, 0.0, 0.0};
  rai_sim::Vec<3> angVelTemp_ = {0.0, 0.0, 0.0};

};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_SINGLEBODYOBJECT_HPP

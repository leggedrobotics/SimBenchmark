//
// Created by kangd on 10.02.18.
//

#ifndef BENCHMARK_OBJECT_HPP
#define BENCHMARK_OBJECT_HPP

#include <btBulletCollisionCommon.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <Eigen/Geometry>
#include <raiSim/math.hpp>

namespace bullet_sim {
namespace object {

class SingleBodyObject {

 public:
  virtual ~SingleBodyObject() = default;

  const Eigen::Map<Eigen::Matrix<double, 4, 1>> getQuaternion();
  void getQuaternion(rai_sim::Vec<4>& quat);
  const Eigen::Map<Eigen::Matrix<double, 3, 3> > getRotationMatrix();
  void getRotationMatrix(rai_sim::Mat<3,3>& rotation);
  const Eigen::Map<Eigen::Matrix<double, 3, 1> > getPosition();
  const Eigen::Map<Eigen::Matrix<double, 3, 1> > getComPosition();
  const Eigen::Map<Eigen::Matrix<double, 3, 1> > getLinearVelocity();
  const Eigen::Map<Eigen::Matrix<double, 3, 1> > getAngularVelocity();

  void getPosition_W(rai_sim::Vec<3>& pos_w);

  bool isVisualizeFramesAndCom() const;

 protected:
  btCollisionShape *collisionShape_;
  btRigidBody *rigidBody_;
  btMotionState *motionState_;

  // from object
  bool visualizeFramesAndCom_ = true;

};

} // object
} // bullet_sim

#endif //BENCHMARK_OBJECT_HPP

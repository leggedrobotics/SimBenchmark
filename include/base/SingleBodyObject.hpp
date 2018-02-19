//
// Created by kangd on 19.02.18.
//

#ifndef BENCHMARK_SINGLEBODYOBJECT_HPP
#define BENCHMARK_SINGLEBODYOBJECT_HPP

#include <Eigen/Geometry>
#include <raiSim/math.hpp>

namespace benchmark {
namespace object {

class SingleBodyObject {

 public:
  virtual const Eigen::Map<Eigen::Matrix<double, 4, 1>> getQuaternion() = 0;
  virtual void getQuaternion(rai_sim::Vec<4>& quat) = 0;
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 3> > getRotationMatrix() = 0;
  virtual void getRotationMatrix(rai_sim::Mat<3,3>& rotation) = 0;
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getPosition() = 0;
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getComPosition() = 0;
  virtual  const Eigen::Map<Eigen::Matrix<double, 3, 1> > getLinearVelocity() = 0;
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getAngularVelocity() = 0;
  virtual void getPosition_W(rai_sim::Vec<3>& pos_w) = 0;

  virtual void setPosition(Eigen::Vector3d originPosition) = 0;
  virtual void setPosition(double x, double y, double z) = 0;
  virtual void setOrientation(Eigen::Quaterniond quaternion) = 0;
  virtual void setOrientation(double w, double x, double y, double z) = 0;
  virtual void setOrientation(Eigen::Matrix3d rotationMatrix) = 0;
  virtual void setOrientationRandom() = 0;
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) = 0;
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) = 0;
  virtual void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) = 0;
  virtual void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) = 0;
  virtual void setExternalForce(Eigen::Vector3d force) = 0;
  virtual void setExternalTorque(Eigen::Vector3d torque) = 0;

  virtual void setRestitutionCoefficient(double restitution) = 0;
  virtual void setFrictionCoefficient(double friction) = 0;

  virtual bool isVisualizeFramesAndCom() const = 0;

 protected:

  // from object
  bool visualizeFramesAndCom_ = true;

};

} // object
} // benchmark

#endif //BENCHMARK_SINGLEBODYOBJECT_HPP

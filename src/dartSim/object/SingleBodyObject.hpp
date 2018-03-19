//
// Created by kangd on 19.03.18.
//

#ifndef DARTSIM_SINGLEBODYOBJECT_HPP
#define DARTSIM_SINGLEBODYOBJECT_HPP

#include <dart/dart.hpp>
#include <base/SingleBodyObject.hpp>

namespace dart_sim {
namespace object {

class SingleBodyObject: public benchmark::object::SingleBodyObject {

 public:
  SingleBodyObject(double mass);
  virtual ~SingleBodyObject();

  const dart::dynamics::SkeletonPtr &getSkeletonPtr() const;

  virtual const Eigen::Map<Eigen::Matrix<double, 4, 1>> getQuaternion();
  virtual void getQuaternion(rai_sim::Vec<4>& quat);
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 3> > getRotationMatrix();
  virtual void getRotationMatrix(rai_sim::Mat<3,3>& rotation);
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getPosition();
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getComPosition();
  virtual  const Eigen::Map<Eigen::Matrix<double, 3, 1> > getLinearVelocity();
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getAngularVelocity();
  virtual void getPosition_W(rai_sim::Vec<3>& pos_w);

  virtual void setPosition(Eigen::Vector3d originPosition);
  virtual void setPosition(double x, double y, double z);
  virtual void setOrientation(Eigen::Quaterniond quaternion);
  virtual void setOrientation(double w, double x, double y, double z);
  virtual void setOrientation(Eigen::Matrix3d rotationMatrix);
  virtual void setOrientationRandom();
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion);
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix);
  virtual void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity);
  virtual void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz);
  virtual void setExternalForce(Eigen::Vector3d force);
  virtual void setExternalTorque(Eigen::Vector3d torque);

  virtual void setRestitutionCoefficient(double restitution);
  virtual void setFrictionCoefficient(double friction);

  virtual bool isVisualizeFramesAndCom() const;

 protected:
  dart::dynamics::SkeletonPtr skeletonPtr_;
  dart::dynamics::ShapePtr shapePtr_;

  double mass_;





};

} // object
} // dart_sim

#endif //BENCHMARK_SINGLEBODYOBJECT_HPP

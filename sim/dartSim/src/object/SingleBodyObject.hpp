//
// Created by kangd on 19.03.18.
//

#ifndef DARTSIM_SINGLEBODYOBJECT_HPP
#define DARTSIM_SINGLEBODYOBJECT_HPP

#include <dart/dart.hpp>

#include "common/math.hpp"
#include "common/interface/SingleBodyObjectInterface.hpp"

#include "Object.hpp"

namespace dart_sim {
namespace object {

class SingleBodyObject: public benchmark::object::SingleBodyObjectInterface,
                        public dart_sim::object::Object {

 public:
  explicit SingleBodyObject(double mass, int id);
  virtual ~SingleBodyObject();

  virtual const Eigen::Map<Eigen::Matrix<double, 4, 1>> getQuaternion() override ;
  virtual void getQuaternion(benchmark::Vec<4>& quat) override ;
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 3> > getRotationMatrix() override ;
  virtual void getRotationMatrix(benchmark::Mat<3,3>& rotation) override ;
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getPosition() override ;
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getComPosition() override ;
  virtual  const Eigen::Map<Eigen::Matrix<double, 3, 1> > getLinearVelocity() override ;
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getAngularVelocity() override ;
  virtual void getPosition_W(benchmark::Vec<3>& pos_w) override ;

  virtual void setPosition(Eigen::Vector3d originPosition) override ;
  virtual void setPosition(double x, double y, double z) override ;
  virtual void setOrientation(Eigen::Quaterniond quaternion) override ;
  virtual void setOrientation(double w, double x, double y, double z) override ;
  virtual void setOrientation(Eigen::Matrix3d rotationMatrix) override ;
  virtual void setOrientationRandom() override ;
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) override ;
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) override ;
  virtual void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) override ;
  virtual void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) override ;
  virtual void setExternalForce(Eigen::Vector3d force) override ;
  virtual void setExternalTorque(Eigen::Vector3d torque) override ;

  virtual void setRestitutionCoefficient(double restitution) override ;
  virtual void setFrictionCoefficient(double friction) override ;

  virtual bool isVisualizeFramesAndCom() const override ;

 protected:
  virtual void dartTf2States();
  virtual void states2DartTf();

  dart::dynamics::ShapePtr shapePtr_;
  dart::dynamics::BodyNodePtr bodyPtr_;

  // pose and velocity
  benchmark::Vec<4> quat_ = {1.0, 0.0, 0.0, 0.0};
  benchmark::Mat<3, 3> rotMat_;
  benchmark::Vec<3> pos_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> linVelTemp_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> angVelTemp_ = {0.0, 0.0, 0.0};

  double mass_;
  int id_;


};

} // object
} // dart_sim

#endif //BENCHMARK_SINGLEBODYOBJECT_HPP

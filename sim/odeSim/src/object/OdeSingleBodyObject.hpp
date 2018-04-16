//
// Created by kangd on 11.02.18.
//

#ifndef ODESIM_SINGLEBODYOBJECT_HPP
#define ODESIM_SINGLEBODYOBJECT_HPP

#include <ode/common.h>
#include <ode/ode.h>
#include <Eigen/Geometry>

#include "common/math.hpp"
#include "common/interface/SingleBodyObjectInterface.hpp"

#include "object/OdeObject.hpp"

namespace ode_sim {
namespace object {

struct MetrialProp {
  double frictionalCoeff = 0.8;
  double restitutionCoeff = 0.0;
};

class OdeSingleBodyObject: public benchmark::object::SingleBodyObjectInterface,
                           public ode_sim::object::OdeObject {

 public:
  OdeSingleBodyObject(const dWorldID worldID, const dSpaceID spaceID);
  virtual ~OdeSingleBodyObject();

  virtual const Eigen::Map<Eigen::Matrix<double, 4, 1>> getQuaternion();
  virtual void getQuaternion(benchmark::Vec<4>& quat);
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 3> > getRotationMatrix();
  virtual void getRotationMatrix(benchmark::Mat<3,3>& rotation);
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getPosition();
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getComPosition();
  virtual  const Eigen::Map<Eigen::Matrix<double, 3, 1> > getLinearVelocity();
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1> > getAngularVelocity();
  virtual void getPosition_W(benchmark::Vec<3>& pos_w);

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
  dWorldID worldID_ = 0;
  dSpaceID spaceID_ = 0;

  dGeomID geometry_ = 0;
  dBodyID body_ = 0;
  dMass mass_;

  MetrialProp matrialProp_;
  
  // pose and velocity
  benchmark::Vec<4> quatTemp_ = {1.0, 0.0, 0.0, 0.0};
  benchmark::Mat<3, 3> rotMatTemp_;
  benchmark::Vec<3> posTemp_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> linVelTemp_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> angVelTemp_ = {0.0, 0.0, 0.0};

  rai::RandomNumberGenerator<double> rn_;
};

} // object
} // ode_sim

#endif //ODESIM_SINGLEBODYOBJECT_HPP

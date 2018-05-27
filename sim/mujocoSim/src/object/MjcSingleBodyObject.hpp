//
// Created by kangd on 18.02.18.
//

#ifndef MUJOCOSIM_SINGLEBODYOBJECT_HPP
#define MUJOCOSIM_SINGLEBODYOBJECT_HPP

#include <Eigen/Geometry>
#include <mujoco.h>
#include <common/UserHandle.hpp>

#include "common/math.hpp"
#include "common/interface/SingleBodyObjectInterface.hpp"

namespace mujoco_sim {
namespace object {

class MjcSingleBodyObject: public benchmark::object::SingleBodyObjectInterface {

 public:
  MjcSingleBodyObject(mjData *data,
                     mjModel *model,
                     int bodyId,
                     int geomId);

  /// see base class for details
  const benchmark::eQuaternion getQuaternion() override ;
  void getQuaternion(benchmark::Vec<4>& quat) override ;
  const benchmark::eRotationMat getRotationMatrix() override ;
  void getRotationMatrix(benchmark::Mat<3,3>& rotation) override ;
  const benchmark::eVector3 getPosition() override ;
  const benchmark::eVector3 getComPosition() override ;
  const benchmark::eVector3 getLinearVelocity() override ;
  const benchmark::eVector3 getAngularVelocity() override ;
  void getPosition_W(benchmark::Vec<3>& pos_w) override ;

  double getKineticEnergy() override ;
  double getPotentialEnergy(const benchmark::Vec<3> &gravity) override ;
  double getEnergy(const benchmark::Vec<3> &gravity) override ;

  const benchmark::eVector3 getLinearMomentum() override;

  void setExternalForce(Eigen::Vector3d force) override ;
  void setExternalTorque(Eigen::Vector3d torque) override ;
  void setFrictionCoefficient(double friction) override ;

  void setCollisionGroupAndMask(int collisionGroup, int collisionMask);

  double getMass();
  bool isMovable() const;

 private:
  /// deprecated overrided functions
  /// ===================================
  void setPosition(Eigen::Vector3d originPosition) override ;
  void setPosition(double x, double y, double z) override ;
  void setOrientation(Eigen::Quaterniond quaternion) override ;
  void setOrientation(double w, double x, double y, double z) override ;
  void setOrientation(Eigen::Matrix3d rotationMatrix) override ;
  void setOrientationRandom() override ;
  void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) override ;
  void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) override ;
  void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) override ;
  void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) override ;

  void setRestitutionCoefficient(double restitution) override ;

  bool isVisualizeFramesAndCom() const override ;
  /// ===================================

  mjtNum *getGeomPosition();
  mjtNum *getGeomRotMat();
  mjtNum *getBodyComPosition();

  mjtNum *getBodyLinearVelocity();
  mjtNum *getBodyAngularVelocity();

  mjtNum getBodyMass();
  mjtNum *getBodyInertia();

  void setGeomFriction(benchmark::Vec<3> friction);

 protected:
  int bodyID_ = 0;    // body id in world
  int geomID_ = 0;    // geometry id in body

  mjData* worldData_;
  mjModel* worldModel_;

  // pose and velocity
  benchmark::Vec<4> quatTemp_ = {1.0, 0.0, 0.0, 0.0};
  benchmark::Mat<3, 3> rotMatTemp_;
  benchmark::Vec<3> posTemp_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> linVelTemp_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> angVelTemp_ = {0.0, 0.0, 0.0};

  benchmark::Vec<3> linearMomentum_ = {0, 0, 0};

  mjtNum temp6D_[6] = {0, 0, 0, 0, 0, 0};
  mjtNum temp3D_[3] = {0, 0, 0};
  mjtNum *tempPtr;

  bool isMovable_ = true;

};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_SINGLEBODYOBJECT_HPP

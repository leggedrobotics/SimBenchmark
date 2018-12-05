//
// Created by kangd on 26.05.18.
//

#ifndef BENCHMARK_BTMBSINGLEBODYOBJECT_HPP
#define BENCHMARK_BTMBSINGLEBODYOBJECT_HPP

#include <common/interface/SingleBodyObjectInterface.hpp>

#include "bullet/b3RobotSimulatorClientAPI_NoGUI.h"
#include "BtMbObject.hpp"

namespace bo = benchmark::object;

namespace bullet_mb_sim {
namespace object {

/**
 * Class for single rigid body object for BtMbSim
 */
class BtMbSingleBodyObject: public bo::SingleBodyObjectInterface,
                            public BtMbObject
{

 public:
  BtMbSingleBodyObject(double mass, b3RobotSimulatorClientAPI_NoGUI *api);

  /// see base class for details
  const benchmark::eQuaternion getQuaternion() override;
  void getQuaternion(benchmark::Vec<4> &quat) override;
  const benchmark::eRotationMat getRotationMatrix() override;
  void getRotationMatrix(benchmark::Mat<3, 3> &rotation) override;
  const benchmark::eVector3 getPosition() override;
  void getPosition_W(benchmark::Vec<3> &pos_w) override;
  const benchmark::eVector3 getComPosition() override;
  const benchmark::eVector3 getLinearVelocity() override;
  const benchmark::eVector3 getAngularVelocity() override;

  void setPosition(Eigen::Vector3d originPosition) override;
  void setPosition(double x, double y, double z) override;
  void setOrientation(Eigen::Quaterniond quaternion) override;
  void setOrientation(double w, double x, double y, double z) override;
  void setOrientation(Eigen::Matrix3d rotationMatrix) override;
  void setOrientationRandom() override;
  void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) override;
  void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) override;
  void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) override;
  void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) override;
  void setExternalForce(Eigen::Vector3d force) override;
  void setExternalTorque(Eigen::Vector3d torque) override;
  void setRestitutionCoefficient(double restitution) override;
  void setFrictionCoefficient(double friction) override;

  double getKineticEnergy() override;
  double getPotentialEnergy(const benchmark::Vec<3> &gravity) override;
  double getEnergy(const benchmark::Vec<3> &gravity) override;
  const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentum() override;

  bool isVisualizeFramesAndCom() const override;

  // only for bullet
  void setFrictionAnchor(bool enableFrictionAnchor);

 protected:
  /**
   * Get body rotation quaternion w.r.t. world frame from Bullet API.
   * This function updates quatTemp_.
   */
  void getBulletQuaternion();

  /**
   * Get body frame origin w.r.t. world frame from Bullet API.
   * This function updates posTemp_
   */
  void getBulletPosition();

  /**
   * Get body velocity w.r.t. world frame from Bullet API.
   * This function updates linVelTemp_.
   */
  void getBulletLinearVelocity();

  /**
   * Get body angular velocity w.r.t. world frame from Bullet API.
   * This function updates angVelTemp_.
   */
  void getBulletAngularVelocity();

  /**
   * Set body frame origin w.r.t. world frame with posTemp_
   * and body rotation quaternion w.r.t. world frame with quatTemp_;
   */
  void setBulletPositionAndQuaternion();

  /**
   * Set body linear and angular velocity w.r.t. world frame with linVelTemp_ and angVelTemp_;
   */
  void setBulletLinearAndAngularVelocity();

  /// Attiributes
  // pose and velocity
  benchmark::Vec<4> quatTemp_ = {1.0, 0.0, 0.0, 0.0};
  benchmark::Mat<3, 3> rotMatTemp_;
  benchmark::Vec<3> posTemp_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> linVelTemp_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> angVelTemp_ = {0.0, 0.0, 0.0};

  // momentum
  benchmark::Vec<3> linearMomentum_ = {0, 0, 0};

  // api
  b3RobotSimulatorClientAPI_NoGUI *api_;

  // inertia
  benchmark::Mat<3,3> localInertia_;

  // mass
  double mass_ = 0;

  // object id (in bullet api)
  int objectId_ = -1;

};

} // object
} // bullet_mb_sim

#endif //BENCHMARK_BTMBSINGLEBODYOBJECT_HPP

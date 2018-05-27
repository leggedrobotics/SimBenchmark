//
// Created by kangd on 19.02.18.
//

#ifndef BENCHMARK_SINGLEBODYOBJECT_HPP
#define BENCHMARK_SINGLEBODYOBJECT_HPP

#include <Eigen/Geometry>
#include "../math.hpp"

namespace benchmark {

typedef Eigen::Map<Eigen::Matrix<double, 4, 1>> eQuaternion;
typedef Eigen::Map<Eigen::Matrix<double, 3, 3>> eRotationMat;
typedef Eigen::Map<Eigen::Matrix<double, 3, 1>> eVector3;

namespace object {

class SingleBodyObjectInterface {

 public:

  /**
   * Get rotation quaternion (w, x, y, z) of object w.r.t. world frame
   * @return Rotation quaternion in Eigen Matrix format
   */
  virtual const eQuaternion getQuaternion() = 0;

  /**
   * Get rotation quaternion (w, x, y, z) of object w.r.t. world frame
   * @param quat    Rotation quaternion in Vec<4> format (output)
   */
  virtual void getQuaternion(Vec<4>& quat) = 0;

  /**
   * Get rotational matrix of object w.r.t. world frame
   * @return Rotational matrix in Eigen Matrix format
   */
  virtual const eRotationMat getRotationMatrix() = 0;

  /**
   * Get rotational matrix of object w.r.t. world frame
   * @param rotation    Rotational matrix in Mat<3,3> format (output)
   */
  virtual void getRotationMatrix(Mat<3,3>& rotation) = 0;

  /**
   * Get position of body frame origin of object w.r.t. world frame.
   * Note that body frame origin may not be on COM
   *
   * @return    Position of body frame origin w.r.t. world frame
   */
  virtual const eVector3 getPosition() = 0;

  /**
   * Get position of body frame origin of object w.r.t. world frame.
   * Note that body frame origin may not be on COM
   *
   * @param pos_w   Position of body frame origin w.r.t. world frame (output)
   */
  virtual void getPosition_W(Vec<3>& pos_w) = 0;

  /**
   * Get position of center of mass frame origin of object w.r.t. world frame.
   * @return    Position of COM w.r.t. world frame
   */
  virtual const eVector3 getComPosition() = 0;

  /**
   * Get linear velocity of object w.r.t. world frame.
   * @return    Linear velocity of object w.r.t. world frame.
   */
  virtual const eVector3 getLinearVelocity() = 0;

  /**
   * Get angular velocity of object w.r.t. world frame.
   * @return    Angular velocity of object w.r.t. world frame.
   */
  virtual const eVector3 getAngularVelocity() = 0;

  /**
   * Set body frame origin w.r.t. world frame
   * @param originPosition  Body frame origin w.r.t. world frame in Vector3d format
   */
  virtual void setPosition(Eigen::Vector3d originPosition) = 0;

  /**
   * Set body frame origin w.r.t. world frame
   * @param x
   * @param y
   * @param z
   */
  virtual void setPosition(double x, double y, double z) = 0;

  /**
   * Set rotation quaternion w.r.t. world frame
   * @param quaternion  Rotation quaternion in Quaterniond format (w, x, y, z)
   */
  virtual void setOrientation(Eigen::Quaterniond quaternion) = 0;

  /**
   * Set rotation quaternion w.r.t. world frame
   * @param w
   * @param x
   * @param y
   * @param z
   */
  virtual void setOrientation(double w, double x, double y, double z) = 0;

  /**
   * Set rotational matrix w.r.t. world frame
   * @param rotationMatrix  Rotational matrix 3x3 in Matrix3d format
   */
  virtual void setOrientation(Eigen::Matrix3d rotationMatrix) = 0;

  /**
   * Set random orientation to object.
   */
  virtual void setOrientationRandom() = 0;

  /**
   * Set body frame origin and quaternion w.r.t. world frame
   * @param originPosition  Body frame origin of object
   * @param quaternion      Body rotation quaternion
   */
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) = 0;

  /**
   * Set body frame origin and rotational matrix w.r.t. world frame
   * @param originPosition  Body frame origin of object
   * @param quaternion      Body rotational matrix
   */
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) = 0;

  /**
   * Set body linear velocity and angular velocity w.r.t. world frame
   * @param linearVelocity      Linear velocity of COM
   * @param angularVelocity     Angular velocity of body
   */
  virtual void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) = 0;

  /**
   * Set body linear velocity and angular velocity w.r.t. world frame   *
   * @param dx  Linear velocity x axis
   * @param dy  Linear velocity y axis
   * @param dz  Linear velocity z axis
   * @param wx  Angular velocity x axis
   * @param wy  Angular velocity y axis
   * @param wz  Angular velocity z axis
   */
  virtual void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) = 0;
  virtual void setExternalForce(Eigen::Vector3d force) = 0;
  virtual void setExternalTorque(Eigen::Vector3d torque) = 0;

  virtual void setRestitutionCoefficient(double restitution) = 0;
  virtual void setFrictionCoefficient(double friction) = 0;

  virtual double getKineticEnergy() = 0;
  virtual double getPotentialEnergy(const benchmark::Vec<3> &gravity) = 0;
  virtual double getEnergy(const benchmark::Vec<3> &gravity) = 0;

  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentum() = 0;

  virtual bool isVisualizeFramesAndCom() const = 0;

 protected:

  // from object
  bool visualizeFramesAndCom_ = true;

};

} // object
} // benchmark

#endif //BENCHMARK_SINGLEBODYOBJECT_HPP

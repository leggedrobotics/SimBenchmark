//
// Created by kangd on 25.05.18.
//

#ifndef BENCHMARK_BTMBARTICULATEDSYSTEM_HPP
#define BENCHMARK_BTMBARTICULATEDSYSTEM_HPP

#include <string>
#include <bullet/SharedMemoryPublic.h>

#include "api/b3RobotSimulatorClientAPI_NoGUI.h"
#include "common/interface/ArticulatedSystemInterface.hpp"

#include "BtMbObject.hpp"

namespace bullet_mb_sim {
namespace object {

/**
 * Articulated system script file type
 */
enum ObjectFileType {
  URDF,
  MJCF,
  SDF
};

class BtMbArticulatedSystem : public benchmark::object::ArticulatedSystemInterface,
                              public BtMbObject
{

 public:

  BtMbArticulatedSystem(std::string filePath,
                          ObjectFileType fileType,
                          bool internalCollision,
                          b3RobotSimulatorClientAPI_NoGUI *api);
  virtual ~BtMbArticulatedSystem();

  /**
   * Get degree of the freedom of the robot:
   * If the robot is floating based then (6 + the number of joints.)
   * If the robot is fixed based then (the number of joints)
   *
   * @return  Degree of freedom of the robot
   */
  int getDOF() override;

  /**
   * Get dimension of the generalized coordinate of the robot:
   * If the robot is floating based then (7 + the number of joints.)
   * If the robot is fixed based then (the number of joints)
   *
   * @return    State dimension of the robot
   */
  int getStateDimension() override;

  /**
   * Get generalized coordinate of robot
   * The dimension of output vector is stateDimension:
   * If the robot is floating based then (base position ; base quaternion ; joint position)
   * If the robot is fixed based then (joint position)
   *
   * @return Eigenvec of generalized coordinate
   */
  const EigenVec getGeneralizedCoordinate() override;

  /**
   * Get generalized velocity of robot
   * The dimension of output vector is degree of freedom:
   * If the robot is floating based then (base linear velocity ; base angular velocity ; joint velocities)
   * If the robot is fixed based then (joint velocities)
   *
   * @return Eigenvec of generalized velocity
   */
  const EigenVec getGeneralizedVelocity() override;

  /**
   * Get generalized force of robot
   * The dimension of output vector is degree of freedom:
   * If the robot is floating based then (base force ; base torque ; joint torque(or force))
   * If the robot is fixed based then (joint torque(or force))
   *
   * @return Eigenvec of generalized force
   */
  const EigenVec getGeneralizedForce() override;

  /**
   * Get generalized coordinate and velocity
   *
   * @param genco   VectorXd of generalized coordinate (output)
   * @param genvel  VectorXd of generalized velocity (output)
   */
  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) override;

  /**
  * Get pose of link w.r.t. world frame
  *
  * @param linkId          linkId
  * @param orientation     orientation of link (output)
  * @param position        position of link (output)
  */
  void getBodyPose(int linkId,
                   benchmark::Mat<3, 3> &orientation,
                   benchmark::Vec<3> &position);

  /**
   * Set generalized coordinate of robot
   * The dimension of input vector is stateDimension:
   * If the robot is floating based then (base position ; base quaternion ; joint position)
   * If the robot is fixed based then (joint position)
   *
   * @param jointState  VectorXd of generalized coordinate
   */
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) override;

  /**
   * Set generalized velocity of robot
   * The dimension of input vector is degree of freedom:
   * If the robot is floating based then (base linear velocity ; base angular velocity ; joint velocities)
   * If the robot is fixed based then (joint velocities)
   *
   * @param jointVel    VectorXd of generalized velocity
   */
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) override;

  /**
   * Set generalized coordinate of robot
   * The dimension of input vector is stateDimension:
   * If the robot is floating based then (base position ; base quaternion ; joint position)
   * If the robot is fixed based then (joint position)
   *
   * @param jointState  Array of generalized coordinate
   */
  void setGeneralizedCoordinate(std::initializer_list<double> jointState) override;

  /**
   * Set generalized velocity of robot
   * The dimension of input vector is degree of freedom:
   * If the robot is floating based then (base linear velocity ; base angular velocity ; joint velocities)
   * If the robot is fixed based then (joint velocities)
   *
   * @param jointVel    Array of generalized velocity
   */
  void setGeneralizedVelocity(std::initializer_list<double> jointVel) override;

  /**
   * Set generalized force of robot
   * The dimension of input vector is degree of freedom:
   * If the robot is floating based then (base force ; base torque ; joint torque(or force))
   * If the robot is fixed based then (joint torque(or force))
   *
   * @return VectorXd of generalized force
   */
  void setGeneralizedForce(const Eigen::VectorXd &tau) override;

  /**
   * Set generalized force of robot
   * The dimension of input vector is degree of freedom:
   * If the robot is floating based then (base force ; base torque ; joint torque(or force))
   * If the robot is fixed based then (joint torque(or force))
   *
   * @return Array of generalized force
   */
  void setGeneralizedForce(std::initializer_list<double> tau) override;

  /**
   * Set generalized coordinate and velocity
   *
   * @param genco   VectorXd of generalized coordinate (input)
   * @param genvel  VectorXd of generalized velocity (input)
   */
  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) override;

  /**
   * deprecated
   */
  void setColor(Eigen::Vector4d color) override {RAIFATAL("setColor is deprecated function")};

  /**
   * Get total mass of the robot
   * @return    total mass of the robot in kg
   */
  double getTotalMass() override;

  /**
   * Get total energy of the robot = kinetic E + potential E
   *
   * @param gravity Gravitational acceleration
   * @return        Total energy of the robot
   */
  double getEnergy(const benchmark::Vec<3> &gravity) override;

  /**
   * Get kinetic energy of the robot
   *
   * @return        Kinetic energy of the robot
   */
  double getKineticEnergy();

  /**
   * Get potential energy of the robot
   *
   * @param gravity Gravitational acceleration
   * @return        Potential energy of the robot
   */
  double getPotentialEnergy(const benchmark::Vec<3> &gravity);

  /**
   * Get linear momentum of the robot in Cartesian space
   *
   * @return    3D output vector of linear momentum
   */
  const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace() override;

 private:
  /**
   * Initialize links's visual object from visual shape data.
   * One link has one visual object.
   *
   * @param objectId    Id of object (in Bullet API)
   * @param data        VisualShapeData of link (output)
   */
  void initVisuals(int objectId, b3VisualShapeData &data);

  /**
   * Initialize link's collision visual object from collision shape info.
   * Note that one link can have multiple collision objects.
   *
   * @param objectId    Id of object (in Bullet API)
   * @param linkId      Id of link
   * @param info        CollisionShapeInformation of link (output)
   */
  void initCollisions(int objectId, int linkId, b3CollisionShapeInformation &info);

  /// Attiributes
  // linear momentum
  benchmark::Vec<3> linearMomentum_;

  // joint idx (controllable joints)
  std::vector<int> ctrbJoints_;

  // link mass
  std::vector<double> mass_;
  std::vector<benchmark::Mat<3,3>> inertia_;    // local inertia (w.r.t com)

  // api
  b3RobotSimulatorClientAPI_NoGUI *api_;

  // object id (in bullet api)
  int objectId_;

  // num of joints
  int numJoints_ = 0;

};

} // object
} // bullet_mb_sim

#endif //BENCHMARK_BTMBARTICULATEDSYSTEM_HPP

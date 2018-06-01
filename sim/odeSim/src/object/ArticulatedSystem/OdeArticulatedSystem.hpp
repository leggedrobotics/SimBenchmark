//
// Created by kangd on 15.04.18.
//

#ifndef BENCHMARK_ODEARTICULATEDSYSTEM_HPP
#define BENCHMARK_ODEARTICULATEDSYSTEM_HPP

#include <ode/ode.h>
#include <string>
#include <iostream>
#include <urdf_parser/urdf_parser.h>

#include "common/interface/ArticulatedSystemInterface.hpp"
#include "common/Configure.hpp"

#include "object/OdeSingleBodyObject.hpp"
#include "object/OdeObject.hpp"
#include "OdeLinkJoint.hpp"

namespace bo = benchmark::object;

namespace ode_sim {
namespace object {

class OdeArticulatedSystem: public bo::ArticulatedSystemInterface,
                            public object::OdeObject {

 public:
  OdeArticulatedSystem(std::string urdfFile,
                       const dWorldID worldID,
                       const dSpaceID spaceID,
                       benchmark::CollisionGroupType collisionGroup,
                       benchmark::CollisionGroupType collisionMask);

  virtual ~OdeArticulatedSystem();

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
 * Set generalized coordinate of robot
 * The dimension of input vector is stateDimension:
 * If the robot is floating based then (base position ; base quaternion ; joint position)
 * If the robot is fixed based then (joint position)
 *
 * @param jointState  VectorXd of generalized coordinate
 */
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) override;

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
   * Get pose of link w.r.t. world frame
   * Note that this pose is link reference frame (origin on joint)
   *
   * @param linkId          linkId
   * @param orientation     orientation of link (output)
   * @param position        position of link (output)
   */
  void getBodyPose(int bodyId,
                   benchmark::Mat<3, 3> &orientation,
                   benchmark::Vec<3> &position);

/**
   * Get list of joint related data (all joint)
   * @return    Vector of Joint data
   */
  const std::vector<Joint *> &getJoints() const;

  /**
   * Get list of link related data (all link)
   * @return    Vector of link data
   */
  const std::vector<Link *> &getLinks() const;

  /**
   * Get total mass of the robot
   * @return    total mass of the robot in kg
   */
  double getTotalMass() override;

  /**
 * Get linear momentum of the robot in Cartesian space
 *
 * @return    3D output vector of linear momentum
 */
  const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace() override;

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


  void getComVelocity_W(int bodyId,
                        benchmark::Vec<3> &velocity);

  void getBodyOmega_W(int bodyId,
                      benchmark::Vec<3> &omega);

  void getComPos_W(int bodyId,
                   benchmark::Vec<3> &comPos);

  void getComRot_W(int bodyId,
                   benchmark::Mat<3, 3> &comOrientation);


  /**
    * not completed
    */
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) override;

  /**
   * not completed
   */
  void setGeneralizedVelocity(std::initializer_list<double> jointVel) override;

 private:
  void init();

  /**
   * Initialize body index, parent index and make link vector (links_)
   *
   * @param link
   */
  void initIdx(Link &link);

  /**
   * Initialize link (ODE geometry, body, inertia and visual objects)
   *
   * @param link                Link class object
   * @param parentRot_w         Link reference frame (joint rorational matrix w.r.t. world)
   * @param parentPos_w         Link reference frame (joint position w.r.t. world)
   * @param visualcollect       (output)
   * @param visualprops         (output)
   * @param collisioncollect    (output)
   * @param collisionprops      (output)
   */
  void initLink(Link &link,
                benchmark::Mat<3, 3> &parentRot_w,
                benchmark::Vec<3> &parentPos_w,
                std::vector<VisualObjectData> &visualcollect,
                std::vector<VisualObjectProperty> &visualprops,
                std::vector<AlternativeVisualObjectData> &collisioncollect,
                std::vector<VisualObjectProperty> &collisionprops);

  /**
   * initialize children joint of link includeing ODE joint objects
   *
   * @param link
   * @param parentRot_w
   * @param parentPos_w
   */
  void initJoints(Link &link, benchmark::Mat<3, 3> &parentRot_w, benchmark::Vec<3> &parentPos_w);

  void processLinkFromUrdf(boost::shared_ptr<const urdf::Link> urdfLink,
                           Link &raiLink,
                           std::vector<std::string> &jointsOrder);

  /**
   * update joint position recursively from generalized coordinate
   *
   * @param link
   * @param parentRot_w
   * @param parentPos_w
   */
  void updateBodyPos(Link &link,
                     benchmark::Mat<3, 3> &parentRot_w,
                     benchmark::Vec<3> &parentPos_w);

  /**
   * update joint velocity recursively from generalized velocity
   *
   * @param link
   * @param parentAngVel_w
   * @param parentLinVel_w
   */
  void updateBodyVelocity(Link &link,
                          benchmark::Vec<3> &parentAngVel_w,
                          benchmark::Vec<3> &parentLinVel_w);


  std::vector<std::string> jointsNames_;

  // the head of links_ is the pointer of rootLink
  std::vector<Link *> links_;
  // the head of joints_ is NOT rootJoint.
  std::vector<Joint *> joints_;

  benchmark::Vec<3> linearMomentum_;

  benchmark::CollisionGroupType collisionGroup_;
  benchmark::CollisionGroupType collisionMask_;

  Link rootLink_;
  Joint rootJoint_;

  dWorldID worldID_ = 0;
  dSpaceID spaceID_ = 0;

};

} // object
} // ode_sim

#endif //BENCHMARK_ODEARTICULATEDSYSTEM_HPP

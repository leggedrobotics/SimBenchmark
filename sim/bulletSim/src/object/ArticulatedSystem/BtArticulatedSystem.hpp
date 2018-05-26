//
// Created by kangd on 25.03.18.
//

#ifndef BULLETSIM_ARTICULATEDSYSTEM_HPP
#define BULLETSIM_ARTICULATEDSYSTEM_HPP

#include <string>
#include <btBulletCollisionCommon.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>

#include "common/interface/ArticulatedSystemInterface.hpp"

#include "URDF/BulletUrdfImporter.h"
#include "URDF/MyMultiBodyCreator.h"
#include "URDF/URDFToBullet.h"
#include "URDF/UrdfParser.h"

#include "object/ArticulatedSystem/URDF/URDFToBullet.h"
#include "object/BtObject.hpp"

namespace bullet_sim {
namespace object {

class BtArticulatedSystem: public bullet_sim::object::BtObject,
                         public benchmark::object::ArticulatedSystemInterface {

 public:
  BtArticulatedSystem(std::string urdfFile, btMultiBodyDynamicsWorld *world);
  virtual ~BtArticulatedSystem();

  /**
   * Get degree of the freedom of the robot:
   * If the robot is floating based then (6 + the number of joints.)
   * If the robot is fixed based then (the number of joints)
   *
   * @return  Degree of freedom of the robot
   */
  virtual int getDOF() override ;

  /**
   * Get dimension of the generalized coordinate of the robot:
   * If the robot is floating based then (7 + the number of joints.)
   * If the robot is fixed based then (the number of joints)
   *
   * @return    State dimension of the robot
   */
  virtual int getStateDimension() override ;

  /**
   * Get generalized coordinate of robot
   * If the robot is floating based then (base position ; base quaternion ; joint position)
   * If the robot is fixed base then (joint position)
   * @return Eigenvec of generalized coordinate (vector dimension = stateDimension)
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
  virtual const EigenVec getGeneralizedForce() override;

  /**
   * Get generalized coordinate and velocity
   *
   * @param genco   VectorXd of generalized coordinate (output)
   * @param genvel  VectorXd of generalized velocity (output)
   */
  virtual void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) override;

  /**
   * Set generalized coordinate of robot
   * The dimension of input vector is stateDimension:
   * If the robot is floating based then (base position ; base quaternion ; joint position)
   * If the robot is fixed based then (joint position)
   *
   * @param jointState  VectorXd of generalized coordinate
   */
  virtual void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) override;

  /**
   * Set generalized coordinate of robot
   * The dimension of input vector is stateDimension:
   * If the robot is floating based then (base position ; base quaternion ; joint position)
   * If the robot is fixed based then (joint position)
   *
   * @param jointState  Array of generalized coordinate
   */
  virtual void setGeneralizedCoordinate(std::initializer_list<double> jointState) override;

  /**
   * Set generalized velocity of robot
   * The dimension of input vector is degree of freedom:
   * If the robot is floating based then (base linear velocity ; base angular velocity ; joint velocities)
   * If the robot is fixed based then (joint velocities)
   *
   * @param jointVel    VectorXd of generalized velocity
   */
  virtual void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) override;

  /**
   * Set generalized velocity of robot
   * The dimension of input vector is degree of freedom:
   * If the robot is floating based then (base linear velocity ; base angular velocity ; joint velocities)
   * If the robot is fixed based then (joint velocities)
   *
   * @param jointVel    Array of generalized velocity
   */
  virtual void setGeneralizedVelocity(std::initializer_list<double> jointVel) override;

  /**
   * Set generalized force of robot
   * The dimension of input vector is degree of freedom:
   * If the robot is floating based then (base force ; base torque ; joint torque(or force))
   * If the robot is fixed based then (joint torque(or force))
   *
   * @return VectorXd of generalized force
   */
  virtual void setGeneralizedForce(const Eigen::VectorXd &tau) override;

  /**
   * Set generalized force of robot
   * The dimension of input vector is degree of freedom:
   * If the robot is floating based then (base force ; base torque ; joint torque(or force))
   * If the robot is fixed based then (joint torque(or force))
   *
   * @return Array of generalized force
   */
  virtual void setGeneralizedForce(std::initializer_list<double> tau) override;

  /**
   * Set generalized coordinate and velocity
   *
   * @param genco   VectorXd of generalized coordinate (input)
   * @param genvel  VectorXd of generalized velocity (input)
   */
  virtual void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) override;

  /**
   * deprecated
   */
  virtual void setColor(Eigen::Vector4d color) override { RAIFATAL("setColor is deprecated function") };

  /**
   * Set internal collision
   *
   * @param Yn  true for internal collision
   */
  void setInternalCollision(bool Yn);

  /**
   * Get pose of link w.r.t. world frame
   *
   * @param bodyId          linkId
   * @param orientation     orientation of link (output)
   * @param position        position of link (output)
   */
  void getBodyPose(int bodyId,
                   benchmark::Mat<3, 3> &orientation,
                   benchmark::Vec<3> &position);

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
   * Get linear momentum of the robot in Cartesian space
   *
   * @return    3D output vector of linear momentum
   */
  const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace() override;

 private:
  void init();
  void initVisuals();
  void initVisualFromLinkCollider(btMultiBodyLinkCollider *linkCollider, int colliderId);
  void initVisualFromCompoundChildList(btCompoundShapeChild *compoundShapeChild,
                                         int id,
                                         int numChild);
  void initVisualFromCollisionShape(btCollisionShape *collisionShape,
                                    btTransform transform,
                                    int id);
  void initVisualFromVisualShape(int id);

  btMultiBodyDynamicsWorld *dynamicsWorld_;
  btMultiBody *multiBody_;
  BulletURDFImporter *importer_;

  std::vector<int> movableLinkIdx_;

  benchmark::Vec<3> linearMomentum_;

  double maxJointTorque_ = 1000.0;
};

} // object
} // rai_sim

#endif //BULLETSIM_ARTICULATEDSYSTEM_HPP

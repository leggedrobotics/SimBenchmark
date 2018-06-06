//
// Created by kangd on 04.04.18.
//

#ifndef DARTSIM_ARTICULATEDSYSTEM_HPP
#define DARTSIM_ARTICULATEDSYSTEM_HPP

#include <dart/dart.hpp>
#include <dart/math/Geometry.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include "common/interface/ArticulatedSystemInterface.hpp"

#include "DartObject.hpp"

namespace dart_sim {
namespace object {

class DartArticulatedSystem: public DartObject,
                             public benchmark::object::ArticulatedSystemInterface {

 public:
  DartArticulatedSystem(std::string urdfFile);
  virtual ~DartArticulatedSystem();

  void updateVisuals();

  virtual const EigenVec getGeneralizedCoordinate() override;

  virtual const EigenVec getGeneralizedVelocity() override;

  /* For floating-base robots, [linearPosition_W, baseRationInQuaternion, joint Angles]
   * For fixed-base robot, [joint angles]
   * The dimension is the DOF+1 for floating-based, and DOF for fixed based. (obtained by getDOF())*/
  virtual void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) override;

  /* For floating-base robots, [linearVelocity_W, angularVelocity_W, jointVelocity]
   * The dimension is the same as dof (obtained with getDOF)*/
  virtual void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) override;

  virtual void setGeneralizedCoordinate(std::initializer_list<double> jointState) override;

  virtual void setGeneralizedVelocity(std::initializer_list<double> jointVel) override;

  virtual void setGeneralizedForce(std::initializer_list<double> tau) override;

  virtual void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) override;

  virtual void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) override;

  virtual void setGeneralizedForce(const Eigen::VectorXd &tau) override;

  void setInternalCollision(bool Yn);

  virtual const EigenVec getGeneralizedForce() override;

  virtual int getDOF() override ;
  
  virtual int getStateDimension() override ;

  virtual void setColor(Eigen::Vector4d color)  override ;

  const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace() override;

  double getTotalMass() override;

  /**
   * Get energy of the robot system.
   * Note. Gravity should be set to world with same value of param &gravity
   *
   * @param gravity this parameter is deprecated
   * @return
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

 private:
  void init();
  void initVisual(dart::dynamics::BodyNode *body);

  benchmark::Vec<3> linearMomentum_;

};

} // object
} // dart_sim

#endif //DARTSIM_ARTICULATEDSYSTEM_HPP

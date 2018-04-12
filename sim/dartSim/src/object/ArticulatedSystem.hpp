//
// Created by kangd on 04.04.18.
//

#ifndef DARTSIM_ARTICULATEDSYSTEM_HPP
#define DARTSIM_ARTICULATEDSYSTEM_HPP

#include <dart/dart.hpp>
#include <dart/math/Geometry.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include "common/interface/ArticulatedSystemInterface.hpp"

#include "Object.hpp"

namespace dart_sim {
namespace object {

class ArticulatedSystem: public Object,
                         public benchmark::object::ArticulatedSystemInterface {

 public:
  ArticulatedSystem(std::string urdfFile);
  virtual ~ArticulatedSystem();

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

  virtual const EigenVec getGeneralizedForce() override;

  virtual int getDOF() override ;

  virtual void setColor(Eigen::Vector4d color)  override ;

 private:
  void init();
  void initVisual(dart::dynamics::BodyNode *body);

};

} // object
} // dart_sim

#endif //DARTSIM_ARTICULATEDSYSTEM_HPP

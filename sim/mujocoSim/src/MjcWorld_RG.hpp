//
// Created by kangd on 18.02.18.
//

#ifndef MUJOCOSIM_WORLD_RG_HPP
#define MUJOCOSIM_WORLD_RG_HPP

#include <raiGraphics/RAI_graphics.hpp>

#include "common/World_RG.hpp"
#include "MjcWorld.hpp"
#include "UserHandle.hpp"

namespace mujoco_sim {

class MjcWorld_RG: public benchmark::World_RG {

 public:

  /* constructor for visualization */
  MjcWorld_RG(int windowWidth,
           int windowHeight,
           float cms,
           const char *modelPath,
           const char *keyPath,
           int flags,
           mujoco_sim::SolverOption solverOption = mujoco_sim::SOLVER_PGS);

  /* constructor for no visualization */
  MjcWorld_RG(const char *modelPath,
           const char *keyPath,
           mujoco_sim::SolverOption solverOption = mujoco_sim::SOLVER_PGS);
  virtual ~MjcWorld_RG();

  //////////////////////////
  /// simulation methods ///
  //////////////////////////
  void setNoSlipParameter(int maxiter);
  void setGravity(Eigen::Vector3d gravity) override ;

  void loop(double realTimeFactor = 1.0);
  void integrate();
  void setTimeStep(double timeStep);

  benchmark::SingleBodyHandle getSingleBodyHandle(int index);
  int getWorldNumContacts() override ;
  int getNumObject() override ;

  const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace();
  double getTotalMass();

  /// the functions below are articulated system related.
  /// ===================================
  const EigenVec getGeneralizedCoordinate();
  const EigenVec getGeneralizedVelocity();

  /* For floating-base robots, [linearPosition_W, baseRationInQuaternion, joint Angles]
   * For fixed-base robot, [joint angles]
   * The dimension is the DOF+1 for floating-based, and DOF for fixed based. (obtained by getDOF())*/
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState);
  /* For floating-base robots, [linearVelocity_W, angularVelocity_W, jointVelocity]
   * The dimension is the same as dof (obtained with getDOF)*/
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel);
  void setGeneralizedCoordinate(std::initializer_list<double> jointState);
  void setGeneralizedVelocity(std::initializer_list<double> jointVel);
  void setGeneralizedForce(std::initializer_list<double> tau);
  void setGeneralizedForce(const Eigen::VectorXd &tau);

  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel);
  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel);

  const EigenVec getGeneralizedForce();
  int getDOF();
  int getStateDimension();
  /// ===================================

 private:
  void initFromModel();

  /// deprecated functions
  void setERP(double erp, double erp2, double frictionErp) override ;
  void integrate(double dt) override ;
  void loop(double dt, double realTimeFactor) override ;

  //////////////////////////////////
  /// adding or removing objects ///
  //////////World////////////////////////

  /// note: use last two parameters as bodyId and geomId rather than collisionGroup and collisionMask
  benchmark::SingleBodyHandle addSphere(double radius,
                                        double mass,
                                        int bodyId,
                                        int geomId) override ;

  benchmark::SingleBodyHandle addBox(double xLength,
                                     double yLength,
                                     double zLength,
                                     double mass,
                                     int bodyId,
                                     int geomId) override ;

  benchmark::SingleBodyHandle addCylinder(double radius,
                                          double height, 
                                          double mass, 
                                          int bodyId,
                                          int geomId) override ;

  benchmark::SingleBodyHandle addCheckerboard(double gridSize,
                                              double xLength,
                                              double yLength,
                                              double reflectanceI,
                                              bo::CheckerboardShape shape,
                                              int bodyId,
                                              int geomId,
                                              int flags = 0) override ;

  benchmark::SingleBodyHandle addCapsule(double radius,
                                         double height,
                                         double mass,
                                         int bodyId,
                                         int geomid) override ;

  mujoco_sim::MjcWorld world_;

  double timeStep_ = 0.01;
};

} // mujoco_sim

#endif //MUJOCOSIM_WORLD_RG_HPP

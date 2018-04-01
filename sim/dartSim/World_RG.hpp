//
// Created by kangd on 16.03.18.
//

#ifndef DARTSIM_WORLD_RG_HPP
#define DARTSIM_WORLD_RG_HPP

#include "base/World_RG.hpp"
#include "World.hpp"

namespace dart_sim {

class World_RG: public benchmark::World_RG {

 public:
  /* constructor for visualization */
  World_RG(int windowWidth,
           int windowHeight,
           float cms,
           int flags = 0);

  /* constructor for no visualization */
  World_RG();
  virtual ~World_RG();

  //////////////////////////////////
  /// adding or removing objects ///
  //////////World////////////////////////
  benchmark::SingleBodyHandle addSphere(double radius,
                                        double mass,
                                        benchmark::CollisionGroupType collisionGroup = 1,
                                        benchmark::CollisionGroupType collisionMask=-1) override ;
  benchmark::SingleBodyHandle addBox(double xLength,
                                     double yLength,
                                     double zLength,
                                     double mass,
                                     benchmark::CollisionGroupType collisionGroup = 1,
                                     benchmark::CollisionGroupType collisionMask = -1) override ;
  benchmark::SingleBodyHandle addCheckerboard(double gridSize,
                                              double xLength,
                                              double yLength,
                                              double reflectanceI,
                                              benchmark::CollisionGroupType collisionGroup = 1,
                                              benchmark::CollisionGroupType collisionMask = -1,
                                              int flags = 0) override ;
  benchmark::SingleBodyHandle addCapsule(double radius,
                                         double height,
                                         double mass,
                                         benchmark::CollisionGroupType collisionGroup = 1,
                                         benchmark::CollisionGroupType collisionMask=-1) override ;

  //////////////////////////
  /// simulation methods ///
  //////////////////////////
  virtual void setGravity(Eigen::Vector3d gravity) override ;

  void loop(double realTimeFactor = 1.0);
  void integrate();
  void setTimeStep(double timeStep);

 private:
  virtual void loop(double dt, double realTimeFactor = 1.0) override ;
  void setERP(double erp, double erp2, double frictionErp) override ;
  virtual void integrate(double dt) override ;

  dart_sim::World world_;

  double timeStep_ = 0.01;

  // solver type
//  SolverOption solverOption_ = SOLVER_SEQUENTIAL_IMPULSE;

};

} // dart_sim

#endif //DARTSIM_WORLD_RG_HPP

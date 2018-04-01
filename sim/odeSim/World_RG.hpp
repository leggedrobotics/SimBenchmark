//
// Created by kangd on 10.02.18.
//

#ifndef ODESIM_WORLD_RG_HPP
#define ODESIM_WORLD_RG_HPP

#define RAI_COLLISION(x) (1<<(x))

#include <raiGraphics/RAI_graphics.hpp>
#include <raiSim/math.hpp>

#include "base/World_RG.hpp"
#include "Configure.hpp"
#include "World.hpp"

namespace ode_sim {

class World_RG: public benchmark::World_RG {

 public:

  /* constructor for visualization */
  World_RG(int windowWidth,
           int windowHeight,
           float cms,
           int flags = 0,
           SolverOption solverOption = SOLVER_STANDARD);

  /* constructor for no visualization */
  World_RG(SolverOption solverOption = SOLVER_STANDARD);
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
  void integrate(double dt) override ;
  void setGravity(Eigen::Vector3d gravity) override ;
  void setERP(double erp, double erp2, double frictionErp) override ;

 private:
  ode_sim::World world_;

};

} // odesim_sim

#endif //ODESIM_WORLD_RG_HPP

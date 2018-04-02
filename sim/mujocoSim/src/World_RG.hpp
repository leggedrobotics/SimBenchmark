//
// Created by kangd on 18.02.18.
//

#ifndef MUJOCOSIM_WORLD_RG_HPP
#define MUJOCOSIM_WORLD_RG_HPP

#include <raiGraphics/RAI_graphics.hpp>

#include "common/World_RG.hpp"
#include "World.hpp"
#include "UserHandle.hpp"

namespace mujoco_sim {

class World_RG: public benchmark::World_RG {

 public:

  /* constructor for visualization */
  World_RG(int windowWidth,
           int windowHeight,
           float cms,
           const char *modelPath,
           const char *keyPath,
           int flags,
           mujoco_sim::SolverOption solverOption);

  /* constructor for no visualization */
  World_RG(const char *modelPath,
           const char *keyPath,
           mujoco_sim::SolverOption solverOption);
  virtual ~World_RG();

  //////////////////////////
  /// simulation methods ///
  //////////////////////////
  void integrate(double dt) override ;

  benchmark::SingleBodyHandle getSingleBodyHandle(int index);

 private:
  void setGravity(Eigen::Vector3d gravity) override ;
  void setERP(double erp, double erp2, double frictionErp) override ;
  void initFromModel();

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
  benchmark::SingleBodyHandle addCylinder(double radius, 
                                          double height, 
                                          double mass, 
                                          benchmark::CollisionGroupType collisionGroup = 1, 
                                          benchmark::CollisionGroupType collisionMask=-1) override ;
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

  mujoco_sim::World world_;
};

} // mujoco_sim

#endif //MUJOCOSIM_WORLD_RG_HPP

//
// Created by kangd on 16.03.18.
//

#ifndef DARTSIM_WORLD_RG_HPP
#define DARTSIM_WORLD_RG_HPP

#include "common/World_RG.hpp"
#include "UserHandle.hpp"
#include "DartWorld.hpp"

namespace dart_sim {

class DartWorld_RG: public benchmark::World_RG {

 public:
  /* constructor for visualization */
  DartWorld_RG(int windowWidth,
               int windowHeight,
               float cms,
               int flags,
               SolverOption solverOption = SOLVER_LCP_DANTZIG,
               CollisionDetectorOption detectorOption = COLLISION_DETECTOR_FCL);

  /* constructor for no visualization */
  DartWorld_RG(SolverOption solverOption = SOLVER_LCP_DANTZIG,
               CollisionDetectorOption detectorOption = COLLISION_DETECTOR_FCL);
  virtual ~DartWorld_RG();

  /////////////////////////////////////
  /// Visualization related methods ///
  /////////////////////////////////////
  virtual void updateFrame() override ;

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
                                              bo::CheckerboardShape shape = bo::BOX_SHAPE,
                                              benchmark::CollisionGroupType collisionGroup = 1,
                                              benchmark::CollisionGroupType collisionMask = -1,
                                              int flags = 0) override ;

  benchmark::SingleBodyHandle addCapsule(double radius,
                                         double height,
                                         double mass,
                                         benchmark::CollisionGroupType collisionGroup = 1,
                                         benchmark::CollisionGroupType collisionMask=-1) override ;

  benchmark::SingleBodyHandle addCylinder(double radius,
                                          double height,
                                          double mass,
                                          benchmark::CollisionGroupType collisionGroup = 1,
                                          benchmark::CollisionGroupType collisionMask=-1) override ;

  ArticulatedSystemHandle addArticulatedSystem(std::string nm,
                                               benchmark::CollisionGroupType collisionGroup = 1,
                                               benchmark::CollisionGroupType collisionMask=-1) ;

  int getNumObject() override ;
  int getWorldNumContacts() override ;

  //////////////////////////
  /// simulation methods ///
  //////////////////////////
  virtual void setGravity(Eigen::Vector3d gravity) override ;

  void loop(double realTimeFactor = 1.0);
  void integrate();
  void setTimeStep(double timeStep);
  void setMaxContacts(int maxcontacts);

 private:
  virtual void loop(double dt, double realTimeFactor) override ;
  void setERP(double erp, double erp2, double frictionErp) override ;
  virtual void integrate(double dt) override ;
  void integrate1(double dt) override;
  void integrate2(double dt) override;

  dart_sim::DartWorld world_;

  double timeStep_ = 0.01;

  // solver type
//  SolverOption solverOption_ = SOLVER_SEQUENTIAL_IMPULSE;

  // list
  std::vector<ArticulatedSystemHandle> asHandles_;

};

} // dart_sim

#endif //DARTSIM_WORLD_RG_HPP

//
// Created by kangd on 10.02.18.
//

#ifndef ODESIM_WORLD_RG_HPP
#define ODESIM_WORLD_RG_HPP

#define RAI_COLLISION(x) (1<<(x))

#include <raiGraphics/RAI_graphics.hpp>

#include "common/math.hpp"
#include "common/World_RG.hpp"
#include "common/Configure.hpp"

#include "UserHandle.hpp"
#include "OdeWorld.hpp"

namespace ode_sim {

class OdeWorld_RG: public benchmark::World_RG {

 public:

  /* constructor for visualization */
  OdeWorld_RG(int windowWidth,
              int windowHeight,
              float cms,
              int flags = 0,
              SolverOption solverOption = SOLVER_STANDARD);

  /* constructor for no visualization */
  OdeWorld_RG(SolverOption solverOption = SOLVER_STANDARD);
  virtual ~OdeWorld_RG();

  /////////////////////////////////////
  /// Visualization related methods ///
  /////////////////////////////////////
//  virtual void visEnd() override ;
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
                                              bo::CheckerboardShape shape = bo::PLANE_SHAPE,
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

  //////////////////////////
  /// simulation methods ///
  //////////////////////////
  void integrate(double dt) override ;
  void setGravity(Eigen::Vector3d gravity) override ;
  void setERP(double erp, double erp2, double frictionErp) override ;

 private:
  ode_sim::OdeWorld world_;

  std::vector<ArticulatedSystemHandle> asHandles_;

  // for debug
  std::vector<rai_graphics::object::Sphere *> bodyOrigins;
  std::vector<rai_graphics::object::Sphere *> jointAxes;
  std::vector<rai_graphics::object::Arrow *> jointArrows;

};

} // odesim_sim

#endif //ODESIM_WORLD_RG_HPP

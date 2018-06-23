//
// Created by kangd on 10.02.18.
//

#ifndef ODESIM_WORLD_RG_HPP
#define ODESIM_WORLD_RG_HPP

#define RAI_COLLISION(x) (1<<(x))

#include <raiGraphics/RAI_graphics.hpp>

#include "common/math.hpp"
#include "common/WorldRG.hpp"
#include "common/Configure.hpp"

#include "UserHandle.hpp"
#include "OdeWorld.hpp"

namespace ode_sim {

class OdeSim: public benchmark::WorldRG {

 public:

  /* constructor for visualization */
  OdeSim(int windowWidth,
         int windowHeight,
         float cms,
         int flags = 0,
         SolverOption solverOption = SOLVER_STANDARD);

  /* constructor for no visualization */
  OdeSim(SolverOption solverOption = SOLVER_STANDARD);
  virtual ~OdeSim();

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
  int getWorldNumContacts() override ;

  //////////////////////////
  /// simulation methods ///
  //////////////////////////
  void integrate(double dt) override ;
  void setGravity(Eigen::Vector3d gravity) override ;
  void setERP(double erp, double erp2, double frictionErp) override ;

  /**
   * Set CFM for constraint force mixing. (softness of contact)
   * @param cfm  0 ~ 1 (ODE default is 1e-10. For benchmark we set default value 0)
   */
  void setCFM(double cfm);

  /**
   * Set solver parameters.
   * @param solverIteration (for quick solver. ODE default is 20)
   */
  void setSolverParameter(int solverIteration);

 private:
  void integrate1(double dt) override;
  void integrate2(double dt) override;
  ode_sim::OdeWorld world_;

  std::vector<ArticulatedSystemHandle> asHandles_;

#ifdef ODESIM_DEBUG
  std::vector<rai_graphics::object::Sphere *> bodyOrigins;
  std::vector<rai_graphics::object::Sphere *> jointAxes;
  std::vector<rai_graphics::object::Arrow *> jointArrows;
#endif

};

} // odesim_sim

#endif //ODESIM_WORLD_RG_HPP

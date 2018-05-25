//
// Created by kangd on 10.02.18.
//

#ifndef BULLETSIM_WORLD_RG_HPP
#define BULLETSIM_WORLD_RG_HPP

#include <raiGraphics/RAI_graphics.hpp>
#include <bullet/btBulletCollisionCommon.h>

#include "common/math.hpp"
#include "common/WorldRG.hpp"
#include "common/Configure.hpp"
#include "UserHandle.hpp"
#include "BtWorld.hpp"

namespace bullet_sim {

/**
 * BtSim is implementation of simulation based on Bullet Physics
 * Only Singlebody simulation is available. Use BtMbSim for multibody simulation.
 */
class BtSim: public benchmark::WorldRG {

 public:

  /// Constructors and desctructors

  /**
   * Constructor with GUI.
   *
   * @param windowWidth     Width of window
   * @param windowHeight    Height of window
   * @param cms             Contact marker size
   * @param solverOption    Bullet solvers (Sequential impulse / NNCG / MLCP PGS etc.)
   * @param flags           Visualization options (see WorldRG for details)
   */
  BtSim(int windowWidth, int windowHeight, float cms,
        SolverOption solverOption = SOLVER_SEQUENTIAL_IMPULSE,
        int flags = 0);

  /**
   * Constructor without GUI (no gui mode)
   *
   * @param solverOption    Bullet solvers (Sequential impulse / NNCG / MLCP PGS etc.)
   */
  BtSim(SolverOption solverOption = SOLVER_SEQUENTIAL_IMPULSE);

  /**
   * Destructor
   */
  virtual ~BtSim();

  /// Visualization related methods
  virtual void visEnd() override ;

  /**
   * implementation of one frame update for BtSim
   */
  virtual void updateFrame() override ;

  /// Add objects
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

  /// Simulation methods
  /**
   * One simulation step with dt.
   * @param dt
   */
  virtual void integrate(double dt) override ;

  /// getters and setters
  /**
   * Getter for the number of objects.
   * Note that one robot system is one object.
   * @return    the number of object in the simulation world.
   */
  int getNumObject() override ;
  int getWorldNumContacts() override ;

  /**
   * Set gravitational acceleration
   * @param gravity 3D vector of gravitational acceleration
   */
  virtual void setGravity(Eigen::Vector3d gravity) override ;

  /**
   * Set ERP for error correcting algorithm (0 ~ 1).
   * Default value is 0.
   * @param nonContactErp   ERP for non-contact
   * @param contactErp      ERP for contact
   * @param frictionErp     ERP for friction
   */
  void setERP(double nonContactErp, double contactErp, double frictionErp) override ;

  /// Setters (BtSim only)
  /**
   * Setter for the number of interations for collision detection
   *
   * @param convexconvex    Num of iterations for convex-convex (default value is 0)
   * @param convexplane     Num of iterations for convex-plane (default value is 0)
   */
  void setMultipointIteration(int convexconvex, int convexplane);

 private:

  /// deprecated ftns
  /**
   * integrate1 is not supported for BtSim
   *
   * @param dt
   */
  void integrate1(double dt) override {};

  /**
   * integrate2 is not supported for BtSim
   *
   * @param dt
   */
  void integrate2(double dt) override {};

  /// Attributes
  // dynamics world
  bullet_sim::BtWorld world_;

  // solver type
  SolverOption solverOption_ = SOLVER_SEQUENTIAL_IMPULSE;

  // handle list
  std::vector<ArticulatedSystemHandle> asHandles_;

};

} // bullet_sim

#endif //BULLETSIM_WORLD_RG_HPP

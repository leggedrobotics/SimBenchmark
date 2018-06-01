//
// Created by kangd on 24.05.18.
//

#ifndef BENCHMARK_BTMBWORLDRG_HPP
#define BENCHMARK_BTMBWORLDRG_HPP

#include <raiGraphics/RAI_graphics.hpp>

#include "common/WorldRG.hpp"
#include "common/math.hpp"
#include "common/Configure.hpp"
#include "common/UserHandle.hpp"

#include "BtMbWorld.hpp"
#include "BtMbUserHandle.hpp"

namespace bullet_mb_sim {

/**
 * BtMbSim is multibody simulator base on Bullet Robotics API
 */
class BtMbSim: public benchmark::WorldRG {
 public:

  /// constructors and destructors
  /**
   * Constructor with GUI.
   * Note that the solver of BtMbSim is Bullet multibody solver.
   *
   * @param windowWidth     Width of window
   * @param windowHeight    Height of window
   * @param cms             Contact marker size
   * @param flags           Visualization options (see WorldRG for details)
   */
  BtMbSim(int windowWidth,
          int windowHeight,
          float cms,
          int flags = 0);

  /**
   * Constructor without GUI (no gui mode)
   */
  BtMbSim();

  /**
   * Destructor
   */
  virtual ~BtMbSim();

  /// Visualization related methods
  /**
   * Implementation of one frame update for BtMbSim
   */
  void updateFrame() override ;

  /// Add object
  /**
   * Add articulated system to simulation from URDF / SDF / MJCF file
   *
   * @param nm              Path to file
   * @param fileType        Type of the file (URDF / SDF / MJCF)
   * @param collisionGroup
   * @param collisionMask
   * @return
   */
  ArticulatedSystemHandle addArticulatedSystem(std::string nm,
                                               object::ObjectFileType fileType,
                                               bool internalCollision = true,
                                               bool maximalCoordinate = false,
                                               benchmark::CollisionGroupType collisionGroup = 1,
                                               benchmark::CollisionGroupType collisionMask = -1);

  /**
   * Add static ground (checkerboard) to simulation
   *
   * @param gridSize        Size of one grid cell
   * @param xLength         Length along x axis
   * @param yLength         Length alogn y axis
   * @param reflectanceI    Reflectance (for gui)
   * @param shape           Collision object shape: Box or 2D static plane
   * @param collisionGroup
   * @param collisionMask
   * @param flags
   * @return
   */
  benchmark::SingleBodyHandle addCheckerboard(double gridSize, double xLength, double yLength, double reflectanceI,
                                              bo::CheckerboardShape shape,
                                              benchmark::CollisionGroupType collisionGroup = 1,
                                              benchmark::CollisionGroupType collisionMask = -1,
                                              int flags = 0) override;

  /**
   * Add sphere to simulation
   *
   * @param radius
   * @param mass
   * @param collisionGroup
   * @param collisionMask
   * @return
   */
  benchmark::SingleBodyHandle addSphere(double radius, double mass,
                                        benchmark::CollisionGroupType collisionGroup = 1,
                                        benchmark::CollisionGroupType collisionMask = -1) override;

  /**
   * Add box to simulation
   *
   * @param xLength
   * @param yLength
   * @param zLength
   * @param mass
   * @param collisionGroup
   * @param collisionMask
   * @return
   */
  benchmark::SingleBodyHandle addBox(double xLength, double yLength, double zLength, double mass,
                                     benchmark::CollisionGroupType collisionGroup = 1,
                                     benchmark::CollisionGroupType collisionMask = -1) override;


  /// pure virtual getter, setter
  /**
   * Getter for the number of objects.
   * Note that one robot system is one object.
   * @return    the number of object in the simulation world.
   */
  int getNumObject() override;

  /**
   * Get number of all contacts existing in a simulation world
   * @return    Number of contacts
   */
  int getWorldNumContacts() override;

  /// pure virtual simulation methods
  /**
   * Set gravitational acceleration
   * @param gravity 3D vector of gravitational acceleration
   */
  void setGravity(Eigen::Vector3d gravity);

  /**
   * Set ERP for error correcting algorithm (0 ~ 1).
   * Default value is 0.
   * @param nonContactErp   ERP for non-contact
   * @param contactErp      ERP for contact
   * @param frictionErp     ERP for friction
   */
  void setERP(double nonContactErp, double contactErp, double frictionErp) override ;

  /// Setters (BtMbSim only)
  /**
   * Setter of simulation timestep
   * @param dt  timestep size in sec
   */
  void setTimeStep(double dt);

  /// Simulation methods
  /**
   * One simulation step
   */
  void integrate();

 private:
  /// deprecated pure virtual function
  /**
   * not supported
   */
  benchmark::SingleBodyHandle addCylinder(double radius, double height, double mass,
                                          benchmark::CollisionGroupType collisionGroup,
                                          benchmark::CollisionGroupType collisionMask) override {};

  /**
   * not supported
   */
  benchmark::SingleBodyHandle addCapsule(double radius, double height, double mass,
                                         benchmark::CollisionGroupType collisionGroup,
                                         benchmark::CollisionGroupType collisionMask) override {};

  /**
   * integrate1 is not supported for BtMbSim
   */
  virtual void integrate1(double dt) override {}; // velocity updates

  /**
   * integrate2 is not supported for BtMbSim
   */
  virtual void integrate2(double dt) override {}; // position updates

  /**
   * integrate with dt parameter is not supported for BtMbSim.
   * Set dt by setTimeStep() and then use integrate()
   */
  virtual void integrate(double dt) override {};

  /// Attributes
  // world
  BtMbWorld world_;

  // handle list
  std::vector<ArticulatedSystemHandle> asHandles_;
};

} // bullet_mb_sim


#endif //BENCHMARK_BTMBWORLDRG_HPP

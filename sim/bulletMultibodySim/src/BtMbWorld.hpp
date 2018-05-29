//
// Created by kangd on 24.05.18.
//

#ifndef BENCHMARK_BTMBWORLD_HPP
#define BENCHMARK_BTMBWORLD_HPP

#include "api/b3RobotSimulatorClientAPI_NoGUI.h"
#include "common/interface/WorldInterface.hpp"

#include "object/BtMbArticulatedSystem.hpp"
#include "object/BtMbCheckerBoard.hpp"
#include "object/BtMbBox.hpp"
#include "object/BtMbSphere.hpp"

namespace bullet_mb_sim {

struct Single3DContactProblem {
  Single3DContactProblem(const btVector3 &point, const btVector3 &normal) {
    point_ = {point.x(), point.y(), point.z()};
    normal_ = {normal.x(), normal.y(), normal.z()};
  };
  Eigen::Vector3d point_;
  Eigen::Vector3d normal_;
};

class BtMbWorld: public benchmark::WorldInterface  {
  friend class BtMbSim;
 public:
  /// constructors and destructors
  explicit BtMbWorld();
  virtual ~BtMbWorld();

  /// overrided function
  void setGravity(const benchmark::Vec<3> &gravity) override;

  /**
  * Getter for the number of objects.
  * Note that one robot system is one object.
  * @return    the number of object in the simulation world.
  */
  int getNumObject() override;

  /// Add object
  /**
   * Add articulated system to simulation from URDF / SDF / MJCF file
   *
   * @param nm              Path to file
   * @param fileType        Type of the file (URDF / SDF / MJCF)
   * @param collisionGroup
   * @param collisionMask
   */
  object::BtMbArticulatedSystem *addArticulatedSystem(std::string nm,
                                                      object::ObjectFileType fileType,
                                                      bool internalCollision,
                                                      benchmark::CollisionGroupType collisionGroup,
                                                      benchmark::CollisionGroupType collisionMask);

  benchmark::object::SingleBodyObjectInterface *addCheckerboard(double gridSize,
                                                                double xLength,
                                                                double yLength,
                                                                double reflectanceI,
                                                                bo::CheckerboardShape shape,
                                                                benchmark::CollisionGroupType collisionGroup,
                                                                benchmark::CollisionGroupType collisionMask) override;

  benchmark::object::SingleBodyObjectInterface *addSphere(double radius,
                                                          double mass,
                                                          benchmark::CollisionGroupType collisionGroup,
                                                          benchmark::CollisionGroupType collisionMask) override;

  benchmark::object::SingleBodyObjectInterface *addBox(double xLength,
                                                       double yLength,
                                                       double zLength,
                                                       double mass,
                                                       benchmark::CollisionGroupType collisionGroup,
                                                       benchmark::CollisionGroupType collisionMask) override;

  /**
  * One simulation step
  */
  void integrate();

 private:
  /// deprecated functions
  /**
   * not supported for BtMbWorld
   */
  benchmark::object::SingleBodyObjectInterface *addCapsule(double radius,
                                                           double height,
                                                           double mass,
                                                           benchmark::CollisionGroupType collisionGroup,
                                                           benchmark::CollisionGroupType collisionMask) override {};
  /**
   * not supported for BtMbWorld
   */
  benchmark::object::SingleBodyObjectInterface *addCylinder(double radius,
                                                            double height,
                                                            double mass,
                                                            benchmark::CollisionGroupType collisionGroup,
                                                            benchmark::CollisionGroupType collisionMask) override {};

  /**
   * integrate1 is not supported for BtMbWorld
   */
  void integrate1(double dt) override {};

  /**
   * integrate2 is not supported for BtMbWorld
   */
  void integrate2(double dt) override {};

  /**
   * integrate with dt is not supported for BtMbWorld.
   * Use setTimeStep(double dt) and integrate() instead.
   */
  void integrate(double dt) override {};

 private:

  /// Attributes
  // Bullet Robotics API
  b3RobotSimulatorClientAPI_NoGUI* api_;

  // object list
  std::vector<object::BtMbObject*> objectList_;

  // contact list
  std::vector<Single3DContactProblem> contactProblemList_;

};

} // bulelt_multibody_sim


#endif //BENCHMARK_BTMBWORLD_HPP

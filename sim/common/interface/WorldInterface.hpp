//
// Created by kangd on 01.04.18.
//

#ifndef BENCHMARK_WORLDINTERFACE_HPP
#define BENCHMARK_WORLDINTERFACE_HPP

#include "SingleBodyObjectInterface.hpp"
#include "common/interface/CheckerboardInterface.hpp"
#include "../Configure.hpp"

namespace bo = benchmark::object;

namespace benchmark {

class WorldInterface {

 public:
  virtual object::SingleBodyObjectInterface *addCheckerboard(double gridSize,
                                                             double xLength,
                                                             double yLength,
                                                             double reflectanceI,
                                                             bo::CheckerboardShape shape,
                                                             benchmark::CollisionGroupType collisionGroup,
                                                             benchmark::CollisionGroupType collisionMask) = 0;

  virtual object::SingleBodyObjectInterface *addSphere(double radius,
                                                       double mass,
                                                       benchmark::CollisionGroupType collisionGroup,
                                                       benchmark::CollisionGroupType collisionMask) = 0;

  virtual object::SingleBodyObjectInterface *addBox(double xLength,
                                                    double yLength,
                                                    double zLength,
                                                    double mass,
                                                    benchmark::CollisionGroupType collisionGroup,
                                                    benchmark::CollisionGroupType collisionMask) = 0;

  virtual object::SingleBodyObjectInterface *addCapsule(double radius,
                                                        double height,
                                                        double mass,
                                                        benchmark::CollisionGroupType collisionGroup,
                                                        benchmark::CollisionGroupType collisionMask) = 0;

  virtual object::SingleBodyObjectInterface *addCylinder(double radius,
                                                         double height,
                                                         double mass,
                                                         CollisionGroupType collisionGroup=1,
                                                         CollisionGroupType collisionMask=-1) = 0;


  /**
   * Set gravitational acceleration
   * @param gravity 3D vector of gravitational acceleration
   */
  virtual void setGravity(const benchmark::Vec<3> &gravity) = 0;

  virtual int getNumObject() = 0;

  /**
   * One simulation step with dt.
   * @param dt  timestep size in sec
   */
  virtual void integrate(double dt) = 0;

  /**
   * Update kinematics with dt. Call this function before applying control input.
   * @param dt  timestep size in sec
   */
  virtual void integrate1(double dt) = 0;

  /**
   * One simulation step after control input.
   * @param dt  timestep size in sec
   */
  virtual void integrate2(double dt) = 0;
};

}


#endif //BENCHMARK_WORLDINTERFACE_HPP

//
// Created by kangd on 14.02.18.
//

#ifndef BENCHMARK_WORLDINTERFACE_HPP
#define BENCHMARK_WORLDINTERFACE_HPP

#include "ObjectInterface.hpp"
#include "Configure.hpp"

namespace benchmark {

class WorldInterface {

 public:

  virtual object::ObjectInterface *addSphere(double radius,
                                             double mass,
                                             CollisionGroupType collisionGroup=1,
                                             CollisionGroupType collisionMask=-1) = 0;
  virtual object::ObjectInterface *addBox(double xLength,
                                          double yLength,
                                          double zLength,
                                          double mass,
                                          CollisionGroupType collisionGroup=1,
                                          CollisionGroupType collisionMask=-1) = 0;
  virtual object::ObjectInterface *addCheckerboard(double gridSize,
                                                   double xLength,
                                                   double yLength,
                                                   double reflectanceI,
                                                   CollisionGroupType collisionGroup=1,
                                                   CollisionGroupType collisionMask=-1) = 0;

  virtual void integrate(double dt) = 0;
//  virtual void setGravity(const btVector3 &gravity) = 0;

};

} // benchmark

#endif //BENCHMARK_WORLDINTERFACE_HPP

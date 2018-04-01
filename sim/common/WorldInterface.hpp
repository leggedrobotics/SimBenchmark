//
// Created by kangd on 01.04.18.
//

#ifndef BENCHMARK_WORLDINTERFACE_HPP
#define BENCHMARK_WORLDINTERFACE_HPP

#include "SingleBodyObjectInterface.hpp"
#include "Configure.hpp"

namespace benchmark {

class WorldInterface {

  virtual object::SingleBodyObjectInterface *addCheckerboard(double gridSize,
                                                             double xLength,
                                                             double yLength,
                                                             double reflectanceI,
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


};

}


#endif //BENCHMARK_WORLDINTERFACE_HPP

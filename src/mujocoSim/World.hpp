//
// Created by kangd on 18.02.18.
//

#ifndef BENCHMARK_WORLD_HPP
#define BENCHMARK_WORLD_HPP

#include <mujoco.h>

#include "Configure.hpp"
#include "object/Sphere.hpp"
#include "object/Box.hpp"
#include "object/Capsule.hpp"
#include "object/CheckerBoard.hpp"

namespace mujoco_sim {

enum SolverOption {
  SOLVER_PGS,
  SOLVER_CG,
  SOLVER_NEWTON
};

class World {

 public:
  World(const char *modelPath, SolverOption solverOption = SOLVER_PGS);
  virtual ~World();

  object::Sphere *addSphere(double radius,
                              double mass,
                              int objectID,
                              benchmark::CollisionGroupType collisionGroup,
                              benchmark::CollisionGroupType collisionMask);
  object::Box *addBox(double xLength,
                        double yLength,
                        double zLength,
                        double mass,
                        int objectID,
                        benchmark::CollisionGroupType collisionGroup,
                        benchmark::CollisionGroupType collisionMask);
  object::CheckerBoard *addCheckerboard(double gridSize,
                                          double xLength,
                                          double yLength,
                                          double reflectanceI,
                                          int objectID,
                                          benchmark::CollisionGroupType collisionGroup,
                                          benchmark::CollisionGroupType collisionMask);
  object::Capsule *addCapsule(double radius,
                                double height,
                                double mass,
                                int objectID,
                                benchmark::CollisionGroupType collisionGroup,
                                benchmark::CollisionGroupType collisionMask);

  mjModel *getWorldModel() const;
  mjData *getWorldData() const;

  void integrate(double dt);

 private:
  mjModel *worldModel_;
  mjData *worldData_;

  // list
  std::vector<object::SingleBodyObject*> objectList_;

};

} // mujoco_sim

#endif //BENCHMARK_WORLD_HPP

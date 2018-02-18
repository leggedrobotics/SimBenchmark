//
// Created by kangd on 18.02.18.
//

#ifndef BENCHMARK_WORLD_HPP
#define BENCHMARK_WORLD_HPP

#include <mujoco.h>

#include "SingleBodyObject.hpp"

namespace mujoco_sim {

enum SolverOption {
  SOLVER_PGS,
  SOLVER_CG,
  SOLVER_NEWTON
};

class World {

 public:
  World(const char *modelPath);
  virtual ~World();

  const std::vector<object::SingleBodyObject *> &getObjectList() const;
  mjModel *getWorldModel() const;
  mjData *getWorldData() const;

 private:
  mjModel *worldModel_;
  mjData *worldData_;

  // list
  std::vector<object::SingleBodyObject*> objectList_;

};

} // mujoco_sim

#endif //BENCHMARK_WORLD_HPP

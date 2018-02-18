//
// Created by kangd on 18.02.18.
//

#ifndef BENCHMARK_WORLD_HPP
#define BENCHMARK_WORLD_HPP

#include <mujoco.h>

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

  mjModel *getWorldModel() const;
  mjData *getWorldData() const;

 private:
  mjModel *worldModel_;
  mjData *worldData_;
};

} // mujoco_sim

#endif //BENCHMARK_WORLD_HPP

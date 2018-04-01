//
// Created by kangd on 20.02.18.
//

#ifndef BENCHMARK_BENCHMARK_HPP
#define BENCHMARK_BENCHMARK_HPP

#include <string>
#include <Eigen/Geometry>

namespace benchmark {

enum Simulator {
  RAI_SIM =     (1 << 0),
  BULLET_SIM =  (1 << 1),
  ODE_SIM =     (1 << 2),
  MUJOCO_SIM =  (1 << 3),
  SIM_END =     (1 << 4)
};

std::string dataPath = "/home/kangd/git/benchmark/data/";

} // benchmark

#endif //BENCHMARK_BENCHMARK_HPP

//
// Created by kangd on 17.02.18.
//

#ifndef BENCHMARK_ROLLING_HPP
#define BENCHMARK_ROLLING_HPP

#include "benchmark.hpp"

namespace benchmark {

enum ForceDirection { FORCE_Y, FORCE_XY };

struct Options {
  // visualization on/off
  bool visualize = true;

  // solver
  std::string solverName;

  // data path
  const std::string parentDir = "rolling/";

  // force-direction
  ForceDirection forceDirection = FORCE_Y;

  // erp
  bool erpYN = true;
};

struct Params {

  /// sim properties
  std::vector<double> lightPosition = {30.0, 0, 10.0};

  /// solver parameters
  double dt = 0.01;     // time step
  double erp = 0.0;

  /// simulation constants
  double m = 1;
  double n = 25;
  double M = 10;
  double g = -9.8;
  double T = 4.0;
  double F = 150;
  double groundMu = 0.5;
  double ballMu = 1.0;
  double boxMu = 0.8;
  double initPenetration = 5e-6;
};

}

#endif //BENCHMARK_ROLLING_HPP

//
// Created by kangd on 20.02.18.
//

#ifndef BENCHMARK_BENCHMARK_HPP
#define BENCHMARK_BENCHMARK_HPP

#include <string>
#include <Eigen/Geometry>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace benchmark {

enum Simulator {
  RAI     = (1 << 0),
  BULLET  = (1 << 1),
  ODE     = (1 << 2),
  MUJOCO  = (1 << 3),
  DART    = (1 << 4)
};

/**
 * options for benchmark test
 */
struct Option {
  // gui on/off
  bool gui = true;

  // print log on/off
  bool log = false;
};

void addDescToOption(po::options_description &desc) {
  desc.add_options()
      ("help", "produce help message")
      ("nogui", "no visualization")
      ("log", po::value<bool>(), "create log files")
      ;
}

} // benchmark

#endif //BENCHMARK_BENCHMARK_HPP

//
// Created by kangd on 27.04.18.
//

#ifndef BENCHMARK_DARTBENCHMARK_HPP
#define BENCHMARK_DARTBENCHMARK_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>
#include "mujocoSim/src/MjcWorld.hpp"
#include "BenchmarkTest.hpp"

namespace po = boost::program_options;

namespace benchmark::mujoco {

/// color of mujoco
float color[3] = {0.2706, 0.4471, 0.6549};

/**
 * options for Dart simulation
 */
struct Option {
  const benchmark::Simulator simulator = benchmark::MUJOCO;
  const std::string simName = "MUJOCO";

  mujoco_sim::SolverOption solverOption = mujoco_sim::SOLVER_PGS;
  std::string solverName = "PGS";

  mujoco_sim::IntegratorOption integratorOption = mujoco_sim::INTEGRATOR_EULER;
  std::string integratorName = "EULER";

  std::string detectorName = "MUJOCO";

  bool noSlip = false;
};
Option options;

std::string getKeypath() {

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../lib/mjpro150/mjkey.txt";

  return keyPath;
}

/**
 * add options to desc
 *
 * @param desc
 */
void addDescToOption(po::options_description &desc) {
  desc.add_options()
      ("solver", po::value<std::string>(), "constraint solver type (pgs / cg / newton)")
      ("integrator", po::value<std::string>(), "integrator type (euler / rk4)")
      ("noslip", "no-slip solver")
      ;
}

/**
 * get option or parameter from arguments
 *
 * @param argc
 * @param argv
 * @param desc
 */
void getOptionsFromArg(int argc, const char **argv, po::options_description &desc) {

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // solver option
  if(vm.count("solver")) {
    if(vm["solver"].as<std::string>().compare("pgs") == 0) {
      options.solverOption = mujoco_sim::SOLVER_PGS;
      options.solverName = "PGS";
    }
    else if (vm["solver"].as<std::string>().compare("cg") == 0) {
      options.solverOption = mujoco_sim::SOLVER_CG;
      options.solverName = "CG";
    }
    else if (vm["solver"].as<std::string>().compare("newton") == 0) {
      options.solverOption = mujoco_sim::SOLVER_NEWTON;
      options.solverName = "NEWTON";
    }
    else {
      RAIFATAL("invalid solver input")
    }
  }

  // integrator
  if(vm.count("integrator")) {
    if(vm["integrator"].as<std::string>().compare("euler") == 0) {
      options.integratorOption = mujoco_sim::INTEGRATOR_EULER;
      options.integratorName = "EULER";
    }
    else if (vm["integrator"].as<std::string>().compare("rk4") == 0) {
      options.integratorOption = mujoco_sim::INTEGRATOR_RK4;
      options.integratorName = "RK4";
    }
    else {
      RAIFATAL("invalid integrator input")
    }
  }

  if(vm.count("noslip")) {
    options.noSlip = true;
    options.solverName += "-NOSLIP";
  }
}

} // benchmark::ode

#endif //BENCHMARK_DARTBENCHMARK_HPP

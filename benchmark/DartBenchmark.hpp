//
// Created by kangd on 27.04.18.
//

#ifndef BENCHMARK_DARTBENCHMARK_HPP
#define BENCHMARK_DARTBENCHMARK_HPP

#include <raiCommon/rai_utils.hpp>
#include "dartSim/src/DartWorld.hpp"

namespace benchmark::dart {

/**
 * options for Dart simulation
 */
struct Option {
  const benchmark::Simulator simulator = benchmark::DART;
  const std::string simName = "DART";

  dart_sim::SolverOption solverOption = dart_sim::SOLVER_LCP_DANTZIG;
  dart_sim::CollisionDetectorOption detectorOption = dart_sim::COLLISION_DETECTOR_BULLET;
  std::string solverName = "DANTZIG";
  std::string detectorName = "BULLET";
};
Option options;

/**
 * add options to desc
 *
 * @param desc
 */
void addDescToOption(po::options_description &desc) {
  desc.add_options()
      ("solver", po::value<std::string>(), "constraint solver type (dantzig/pgs)")
      ("detector", po::value<std::string>(), "collision detector type (fcl/dart/bullet/ode)")
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
    if(vm["solver"].as<std::string>().compare("dantzig") == 0) {
      options.solverOption = dart_sim::SOLVER_LCP_DANTZIG;
      options.solverName = "DANTZIG";
    }
    else if (vm["solver"].as<std::string>().compare("pgs") == 0) {
      options.solverOption = dart_sim::SOLVER_LCP_PGS;
      options.solverName = "PGS";
    }
    else {
      RAIFATAL("invalid solver input")
    }
  }

  // detector option
  if(vm.count("detector")) {
    if(vm["detector"].as<std::string>().compare("fcl") == 0) {
      options.detectorOption = dart_sim::COLLISION_DETECTOR_FCL;
      options.detectorName = "FCL";
    }
    else if (vm["detector"].as<std::string>().compare("bullet") == 0) {
      options.detectorOption = dart_sim::COLLISION_DETECTOR_BULLET;
      options.detectorName = "BULLET";
    }
    else if (vm["detector"].as<std::string>().compare("ode") == 0) {
      options.detectorOption = dart_sim::COLLISION_DETECTOR_ODE;
      options.detectorName = "ODE";
    }
    else if (vm["detector"].as<std::string>().compare("dart") == 0) {
      options.detectorOption = dart_sim::COLLISION_DETECTOR_DART;
      options.detectorName = "DART";
    }
    else {
      RAIFATAL("invalid detector input")
    }
  }
}

} // benchmark::dart

#endif //BENCHMARK_DARTBENCHMARK_HPP

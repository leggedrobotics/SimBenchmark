//
// Created by kangd on 27.04.18.
//

#ifndef BENCHMARK_DARTBENCHMARK_HPP
#define BENCHMARK_DARTBENCHMARK_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>
#include "odeSim/src/OdeWorld.hpp"

namespace po = boost::program_options;

namespace benchmark::ode {

/// color of ode
float color[3] = {0.8588, 0.5176, 0.2392};

/**
 * options for Dart simulation
 */
struct Option {
  const benchmark::Simulator simulator = benchmark::ODE;
  const std::string simName = "ODE";

  ode_sim::SolverOption solverOption = ode_sim::SOLVER_STANDARD;
  std::string solverName = "STANDARD";

  std::string detectorName = "ODE";
  std::string integratorName = "ODE";
};
Option options;

/**
 * add options to desc
 *
 * @param desc
 */
void addDescToOption(po::options_description &desc) {
  desc.add_options()
      ("solver", po::value<std::string>(), "constraint solver type (std / quick)")
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

  // help option
  if(vm.count("solver")) {
    if(vm["solver"].as<std::string>().compare("std") == 0) {
      options.solverOption = ode_sim::SOLVER_STANDARD;
      options.solverName = "STANDARD";
    }
    else if (vm["solver"].as<std::string>().compare("quick") == 0) {
      options.solverOption = ode_sim::SOLVER_QUICK;
      options.solverName = "QUICK";
    }
    else {
      RAIFATAL("invalid solver input")
    }
  }
}

} // benchmark::ode

#endif //BENCHMARK_DARTBENCHMARK_HPP

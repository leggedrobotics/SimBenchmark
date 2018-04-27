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

/**
 * options for Dart simulation
 */
struct Option {
  ode_sim::SolverOption solverOption = ode_sim::SOLVER_STANDARD;
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
void getParamsFromArg(int argc, const char *argv[], po::options_description &desc) {

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // help option
  if(vm.count("solver")) {
    if(vm["solver"].as<std::string>().compare("std") == 0) {
      options.solverOption = ode_sim::SOLVER_STANDARD;
    }
    else if (vm["solver"].as<std::string>().compare("quick") == 0) {
      options.solverOption = ode_sim::SOLVER_QUICK;
    }
    else {
      RAIFATAL("invalid solver input")
    }
  }
}

} // benchmark::ode

#endif //BENCHMARK_DARTBENCHMARK_HPP

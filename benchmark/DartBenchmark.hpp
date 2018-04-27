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
  dart_sim::SolverOption solverOption = dart_sim::SOLVER_LCP_DANTZIG;
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
    if(vm["solver"].as<std::string>().compare("dantzig") == 0) {
      options.solverOption = dart_sim::SOLVER_LCP_DANTZIG;
    }
    else if (vm["solver"].as<std::string>().compare("pgs") == 0) {
      options.solverOption = dart_sim::SOLVER_LCP_PGS;
    }
    else {
      RAIFATAL("invalid solver input")
    }
  }
}

} // benchmark::dart

#endif //BENCHMARK_DARTBENCHMARK_HPP

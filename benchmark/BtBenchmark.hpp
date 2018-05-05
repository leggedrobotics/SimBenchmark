//
// Created by kangd on 27.04.18.
//

#ifndef BENCHMARK_DARTBENCHMARK_HPP
#define BENCHMARK_DARTBENCHMARK_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>
#include "bulletSim/src/BtWorld.hpp"

namespace po = boost::program_options;

namespace benchmark::bullet {

/**
 * options for Dart simulation
 */
struct Option {
  const benchmark::Simulator simulator = benchmark::BULLET;
  const std::string simName = "BULLET";

  bullet_sim::SolverOption solverOption = bullet_sim::SOLVER_SEQUENTIAL_IMPULSE;
  std::string solverName = "SEQUENCEIMPULSE";
};
Option options;

/**
 * add options to desc
 *
 * @param desc
 */
void addDescToOption(po::options_description &desc) {
  desc.add_options()
      ("solver", po::value<std::string>(), "constraint solver type (seqimp / nncg / pgs / dantzig / lemke / multibody)")
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
    if(vm["solver"].as<std::string>().compare("seqimp") == 0) {
      options.solverOption = bullet_sim::SOLVER_SEQUENTIAL_IMPULSE;
      options.solverName = "SEQUENCEIMPULSE";
    }
    else if (vm["solver"].as<std::string>().compare("nncg") == 0) {
      options.solverOption = bullet_sim::SOLVER_NNCG;
      options.solverName = "NNCG";
    }
    else if (vm["solver"].as<std::string>().compare("pgs") == 0) {
      options.solverOption = bullet_sim::SOLVER_MLCP_PGS;
      options.solverName = "MLCPPGS";
    }
    else if (vm["solver"].as<std::string>().compare("dantzig") == 0) {
      options.solverOption = bullet_sim::SOLVER_MLCP_DANTZIG;
      options.solverName = "MLCPDANTZIG";
    }
    else if (vm["solver"].as<std::string>().compare("lemke") == 0) {
      options.solverOption = bullet_sim::SOLVER_MLCP_LEMKE;
      options.solverName = "MLCPLEMKE";
    }
    else if (vm["solver"].as<std::string>().compare("multibody") == 0) {
      options.solverOption = bullet_sim::SOLVER_MULTI_BODY;
      options.solverName = "MULTIBODY";
    }
    else {
      RAIFATAL("invalid solver input")
    }
  }
}

} // benchmark::bullet

#endif //BENCHMARK_DARTBENCHMARK_HPP

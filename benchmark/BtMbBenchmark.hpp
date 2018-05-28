//
// Created by kangd on 27.04.18.
//

#ifndef BENCHMARK_DARTBENCHMARK_HPP
#define BENCHMARK_DARTBENCHMARK_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace benchmark::bullet {

/// color of bullet
float color[3] = {0.6667, 0.2745, 0.2627};

/**
 * options for Dart simulation
 */
struct Option {
  const benchmark::Simulator simulator = benchmark::BULLET;
  const std::string simName = "BULLET";

  std::string solverName = "MULTIBODY";
  std::string detectorName = "BULLET";
  std::string integratorName = "BULLET";

  bool maximalCoordinate = false;
};
Option options;

/**
 * add options to desc
 *
 * @param desc
 */
void addDescToOption(po::options_description &desc) {
  desc.add_options()
      ("maxCo", "Maximal coordinate for multibody objects")
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
  if(vm.count("maxCo")) {
    options.maximalCoordinate = true;
  }
}

} // benchmark::bullet

#endif //BENCHMARK_DARTBENCHMARK_HPP

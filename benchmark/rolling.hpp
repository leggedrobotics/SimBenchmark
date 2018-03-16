//
// Created by kangd on 17.02.18.
//

#ifndef BENCHMARK_ROLLING_HPP
#define BENCHMARK_ROLLING_HPP

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "benchmark.hpp"

namespace po = boost::program_options;
namespace ru = rai::Utils;

namespace rolling_benchmark {

/// data path
const std::string parentDir = "rolling";
enum ForceDirection { FORCE_Y, FORCE_XY };

/// arg options
po::options_description desc;

/// options
struct Options {
  // visualization on/off
  bool visualize = true;

  // solver
  std::string solverName;

  // force-direction
  ForceDirection forceDirection = FORCE_Y;

  // erp
  bool erpYN = true;
};

/// params
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

Options options;
Params params;

void loggerSetup(std::string path, std::string name) {
  // logger
  ru::logger->setCreatePathDir(true);
  ru::logger->setLogPath(path);
  ru::logger->setLogFileName(name);
  ru::logger->setOptions(ru::ONEFILE_FOR_ONEDATA);
  ru::logger->addVariableToLog(3, "velbox", "linear velocity of box");
  ru::logger->addVariableToLog(3, "velball", "linear velocity of ball");
  ru::logger->addVariableToLog(3, "posbox", "position of box");
  ru::logger->addVariableToLog(3, "posball", "position of ball");

  // timer
  std::string timer = name + "timer";
  ru::timer->setCreatePathDir(true);
  ru::timer->setLogPath(path);
  ru::timer->setLogFileName(timer);
}

void getParamsFromYAML(char *yamlfile) {
  /// parameters from yaml
  YAML::Node yaml = YAML::LoadFile(yamlfile);

  // simulation options
  params.dt = yaml["options"]["dt"].as<double>();
  if(yaml["options"]["force_direction"].as<std::string>().compare("xy") == 0) {
    options.forceDirection = FORCE_XY;
  } else if(yaml["options"]["force_direction"].as<std::string>().compare("y") == 0) {
    options.forceDirection = FORCE_Y;
  } else {
    RAIFATAL("invalid force-direction");
  }
  options.erpYN = yaml["options"]["erp_on"].as<bool>();

  // sim properties
  YAML::Node props = yaml["sim_properties"];
  params.lightPosition = props["light_position"].as<std::vector<double>>();

  // simulation constants
  YAML::Node constant = yaml["constant"];
  params.m = constant["m"].as<double>();
  params.n = constant["n"].as<double>();
  params.M = constant["M"].as<double>();
  params.g = constant["g"].as<double>();
  params.T = constant["T"].as<double>();
  params.F = constant["F"].as<double>();
  params.groundMu = constant["mu_ground"].as<double>();
  params.ballMu = constant["mu_ball"].as<double>();
  params.boxMu = constant["mu_box"].as<double>();
  params.initPenetration = constant["penentration0"].as<double>();
}

void getParamsFromArg(int argc, const char *argv[]) {
  /// parameter from arguments
  po::options_description genericdesc("for all");
  genericdesc.add_options()
      ("help", "produce help message")
      ("nogui", "no visualization")
      ("erp-on", po::value<bool>(), "erp on (true / false)")
      ("dt", po::value<double>(), "time step for simulation (e.g. 0.01)")
      ("force-direction", po::value<std::string>(), "applied force direction (y / xy)")
      ;
  desc.add(genericdesc);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // help option
  if(vm.count("help")) {
    RAIINFO(desc)
  }

  // nogui option
  if(vm.count("nogui")) {
    options.visualize = false;
  }

  // dt option
  if(vm.count("dt")) {
    params.dt = vm["dt"].as<double>();
  }

  // force direction
  if(vm.count("force-direction")) {
    if(vm["force-direction"].as<std::string>().compare("xy") == 0) {
      options.forceDirection = FORCE_XY;
    } else if(vm["force-direction"].as<std::string>().compare("y") == 0) {
      options.forceDirection = FORCE_Y;
    } else {
      RAIFATAL("invalid force-direction")
    }
  }

  // erp
  if(vm.count("erp-on")) {
    if(vm["erp-on"].as<bool>()) {
      options.erpYN = true;
    } else {
      options.erpYN = false;
    }
  }
}

} // benchmark

#endif //BENCHMARK_ROLLING_HPP

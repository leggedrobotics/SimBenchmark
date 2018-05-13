//
// Created by kangd on 10.05.18.
//

#ifndef BENCHMARK_THOUSANDBENCHMARK_HPP
#define BENCHMARK_THOUSANDBENCHMARK_HPP

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

/**
 * Thousand test is for testing hard-contact constraint.
 * The test focuses on:
 *
 * 1. compliance of hard-contact constraint
 *
 * Please read docs for more details
 */

namespace po = boost::program_options;
namespace ru = rai::Utils;

namespace benchmark::thousand {

/**
 * options for building simulation
 */
struct Option: benchmark::Option {
  // erp
  bool erpYN = false;

  // time step
  double dt = 0.001;

  // simulation time
  double T = 15.0;

  // timer
  bool timer = false;
};
Option options;

/**
 * parameter for building simulation
 * this can be set by YAML file
 */
struct Parameter {

  // sim properties
  double lightPosition[3] = {30.0, 0, 10.0};

  // solver parameters
  double erp = 0.2;
  double erp2 = 0.1;              // for bullet
  double erpFriction = 0.01;      // for bullet

  // simulation parameters
  /// (num of object) = n * n * n
  int n = 6;

  double gap = 0.21;
  double perturbation = 0.001;
  double H = 1.101;

  double ballR = 0.1;     // radius of ball
  double ballM = 10;    // mass of ball

  int randomSeed = 42;
};
Parameter params;

/**
 * get XML file path for Mujoco
 *
 * @return xml path in string
 */
std::string getMujocoXMLpath() {

  int ncubic = params.n * params.n * params.n;

  std::string xmlPath(__FILE__);
  while (xmlPath.back() != '/')
    xmlPath.erase(xmlPath.size() - 1, 1);
  xmlPath += "../res/mujoco/Thousand/sphere" + std::to_string(ncubic) + ".xml";

  return xmlPath;
}

/**
 * get YAML file path for parameter
 *
 * @return yaml path in string
 */
std::string getYamlpath() {

  std::string yamlPath(__FILE__);
  while (yamlPath.back() != '/')
    yamlPath.erase(yamlPath.size() - 1, 1);
  yamlPath += "./yaml/thousand.yaml";

  return yamlPath;
}

/**
 * get log file directory path of test result
 *
 * @param erp
 * @return log directory path in string
 */
std::string getLogDirpath(bool erpYN,
                          std::string simulation,
                          std::string solver,
                          double dt) {

  std::string dirPath(__FILE__);
  while (dirPath.back() != '/')
    dirPath.erase(dirPath.size() - 1, 1);

  dirPath += "../data/thousand/erp=" + std::to_string(erpYN)
      + "/" + simulation
      + "/" + solver
      + "/" + std::to_string(dt);

  return dirPath;
}

/**
 * add options to desc
 *
 * @param desc
 */
void addDescToOption(po::options_description &desc) {
  benchmark::addDescToOption(desc);
  desc.add_options()
      ("erp-on", po::value<bool>(), "erp on (true / false)")
      ("dt", po::value<double>(), "time step for simulation (e.g. 0.01)")
      ("T", po::value<double>(), "simulation time (e.g. 15)")
      ("timer", "timer on for speed check")
      ;
}

/**
 * get options from arguments
 *
 * @param argc
 * @param argv
 * @param desc
 */
void getOptionsFromArg(int argc, const char *argv[], po::options_description &desc) {

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // help option
  if(vm.count("help")) {
    std::cout << desc << std::endl;
    exit(0);
  }

  // log option
  if(vm.count("log")) {
    options.log = true;
  }

  // timer option
  if(vm.count("timer")) {
    options.timer = true;
  }

  // nogui option
  if(vm.count("nogui")) {
    options.gui = false;
  }

  // save video
  if(vm.count("video")) {
    RAIFATAL_IF(!options.gui, "GUI should be on to save a video")
    options.saveVideo = true;
  }

  // dt option
  if(vm.count("dt")) {
    options.dt = vm["dt"].as<double>();
  }

  // T option
  if(vm.count("T")) {
    options.T = vm["T"].as<double>();
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

/**
 * get params from YAML
 *
 * @param yamlfile
 */
void getParamsFromYAML(const char *yamlfile, benchmark::Simulator simulator) {
  /// parameters from yaml
  YAML::Node yaml = YAML::LoadFile(yamlfile);

  // sim properties
  YAML::Node props = yaml["sim_properties"];
  params.lightPosition[0] = props["light_position"].as<std::vector<double>>()[0];
  params.lightPosition[1] = props["light_position"].as<std::vector<double>>()[1];
  params.lightPosition[2] = props["light_position"].as<std::vector<double>>()[2];

  // simulation constants
  YAML::Node constant = yaml["constant"];
  params.ballR = constant["ballR"].as<double>();
  params.ballM = constant["ballM"].as<double>();
  params.gap = constant["gap"].as<double>();
  params.perturbation = constant["perturbation"].as<double>();
  params.H = constant["H"].as<double>();
  params.n = constant["n"].as<int>();

  // solver parameters
  YAML::Node solver_params = yaml["solver_params"];

  switch (simulator) {
    case benchmark::RAI:
      params.erp = solver_params["raiSim"]["erp"].as<double>();
      break;
    case benchmark::BULLET:
      params.erp = solver_params["bullet"]["erp"].as<double>();
      params.erp2 = solver_params["bullet"]["erp2"].as<double>();
      params.erpFriction = solver_params["bullet"]["erp_friction"].as<double>();
      break;
    case benchmark::ODE:
      params.erp = solver_params["ode"]["erp"].as<double>();
      break;
    case benchmark::MUJOCO:
      break;
    case benchmark::DART:
      break;
    default:
      RAIFATAL("invalid simulator value")
  }
}

/**
 * set up logger and timer log
 *
 * @param path directory path of log files
 * @param name name of log file
 */
void loggerSetup(std::string path, std::string name) {
  // timer
  ru::logger->setLogPath(path);
  ru::logger->setLogFileName(name);
  ru::logger->setOptions(ru::ONEFILE_FOR_ONEDATA);
  ru::logger->addVariableToLog(1, "error", "penetration error");

  std::string timer = name + "timer";
  ru::timer->setLogPath(path);
  ru::timer->setLogFileName(timer);
}

} // benchmark::rolling


#endif //BENCHMARK_THOUSANDBENCHMARK_HPP

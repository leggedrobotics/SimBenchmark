//
// Created by kangd on 11.05.18.
//

#ifndef BENCHMARK_BOUNCINGBENCHMARK_HPP
#define BENCHMARK_BOUNCINGBENCHMARK_HPP

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

namespace po = boost::program_options;
namespace ru = rai::Utils;

namespace benchmark::bouncing {

/**
 * options for bouncing simulation
 */
struct Option: benchmark::Option {
  // erp
  bool erpYN = false;

  // time step
  double dt = 0.01;

  // restitution coefficient
  double e = 1.0;
};
Option options;

/**
 * parameter for bouncing simulation
 * this can be set by YAML file
 */
struct Parameter {

  // sim properties
  double lightPosition[3] = {30.0, 0, 10.0};

  // solver parameters
  double erp = 0.2;
  double erp2 = 0.1;            // for bullet
  double erpFriction = 0.1;     // for bullet

  // simulation parameters
  double m = 1500;              // mass of ball
  int n = 1;                 // (num obj) = n x n
  double H = 10;
  double R = 0.5;               // radius of ball
  double g = -9.8;
  double T = 10;

  double mu_ground = 0;
  double mu_ball = 0;

  /// note
  /// 1. (frictional coeff A-B) = (friction coeff of A) x (friction coeff of B)             - Bullet & ODE
  /// 2. (frictional coeff A-B) = max of (friction coeff of A) and (friction coeff of B)    - Mujoco
  /// 3. (frictional coeff A-B) = min of (friction coeff of A) and (friction coeff of B)    - Dart
  /// 4. (frictional coeff A-B)                                                             - Rai
};
Parameter params;

/**
 * get XML file path for Mujoco
 *
 * @return urdf path in string
 */
std::string getMujocoXMLpath() {

  const int nsq = (params.n) * (params.n);
  std::string xmlPath(__FILE__);
  while (xmlPath.back() != '/')
    xmlPath.erase(xmlPath.size() - 1, 1);
  xmlPath += "../res/mujoco/Bouncing/bouncing" + std::to_string(nsq) + ".xml";

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
  yamlPath += "./yaml/bouncing.yaml";

  return yamlPath;
}

/**
 * get log file directory path of test result
 *
 * @param erp
 * @return log directory path in string
 */
std::string getLogDirpath(bool erpYN,
                          double restCoeff,
                          std::string simulation,
                          std::string solver,
                          double dt) {

  std::string dirPath(__FILE__);
  while (dirPath.back() != '/')
    dirPath.erase(dirPath.size() - 1, 1);

  dirPath += "../data/bouncing/erp=" + std::to_string(erpYN)
      + "-res=" + std::to_string(restCoeff)
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
      ("e", po::value<double>(), "restitution coefficient (0-1)")
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

  // restitution coeff
  if(vm.count("e")) {
    options.e = vm["e"].as<double>();
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
  params.m = constant["m"].as<double>();
  params.n = constant["n"].as<int>();
  params.g = constant["g"].as<double>();
  params.R = constant["R"].as<double>();
  params.mu_ground = constant["mu_ground"].as<double>();
  params.mu_ball = constant["mu_ball"].as<double>();
  params.T = constant["T"].as<double>();

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
  // logger
  ru::logger->setLogPath(path);
  ru::logger->setLogFileName(name);
  ru::logger->setOptions(ru::ONEFILE_FOR_ONEDATA);
  ru::logger->addVariableToLog(1, "energy", "energy of balls");
}

} // benchmark::bouncing

#endif //BENCHMARK_BOUNCINGBENCHMARK_HPP

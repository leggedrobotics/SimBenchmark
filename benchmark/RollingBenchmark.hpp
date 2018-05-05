//
// Created by kangd on 17.02.18.
//

#ifndef BENCHMARK_ROLLING_HPP
#define BENCHMARK_ROLLING_HPP

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

/**
 * Rolling test is for testing frictional contact solving.
 * The error is measured by comparing the simulation with analytical solution.
 * The test focuses on:
 *
 * 1. Frictional cone (diagonal, elliptic)
 * 2. The accuracy of frictional contact simulation
 * 3. The violation of hard-contact constraint (penetration)
 */

namespace po = boost::program_options;
namespace ru = rai::Utils;

namespace benchmark::rolling {

enum ForceDirection {
  FORCE_Y,    // force along the y axis
  FORCE_XY    // force along the diagonal direction
};

/**
 * options for rolling simulation
 */
struct Option: benchmark::Option {
  // force-direction Y/XY
  ForceDirection forceDirection = FORCE_Y;

  // erp
  bool erpYN = false;

  // time step
  double dt = 0.01;
};
Option options;

/**
 * parameter for rolling simulation
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

  /// note
  /// 1. (frictional coeff A-B) = (friction coeff of A) x (friction coeff of B)             - Bullet & ODE
  /// 2. (frictional coeff A-B) = max of (friction coeff of A) and (friction coeff of B)   - Mujoco
  /// 3. (frictional coeff A-B)                                                             - Rai
};
Parameter params;

/**
 * get XML file path for Mujoco
 *
 * @param rowNum # of row
 * @return urdf path in string
 */
std::string getMujocoXMLpath() {

  std::string xmlPath(__FILE__);
  while (xmlPath.back() != '/')
    xmlPath.erase(xmlPath.size() - 1, 1);
  xmlPath += "../res/mujoco/Rolling/rolling.xml";

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
  yamlPath += "../res/YAML/rolling.yaml";

  return yamlPath;
}

/**
 * get log file directory path of test result
 *
 * @param erp
 * @return log directory path in string
 */
std::string getLogDirpath(bool erpYN,
                          ForceDirection forceDirection,
                          std::string simulation,
                          std::string solver,
                          double dt) {

  std::string dirPath(__FILE__);
  while (dirPath.back() != '/')
    dirPath.erase(dirPath.size() - 1, 1);

  dirPath += "../data/rolling/erp=" + std::to_string(erpYN)
      + "-dir=" + std::to_string(forceDirection)
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
      ("force-direction", po::value<std::string>(), "applied force direction (y / xy)")
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
    options.log = vm["log"].as<bool>();
  }

  // nogui option
  if(vm.count("nogui")) {
    options.gui = false;
  }

  // dt option
  if(vm.count("dt")) {
    options.dt = vm["dt"].as<double>();
  }

  // force direction
  if(vm.count("force-direction")) {
    if(vm["force-direction"].as<std::string>().compare("xy") == 0) {
      options.forceDirection = FORCE_XY;
    } else if(vm["force-direction"].as<std::string>().compare("y") == 0) {
      options.forceDirection = FORCE_Y;
    } else {
      RAIFATAL("invalid force-direction (should be xy or y)")
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
  params.n = constant["n"].as<double>();
  params.M = constant["M"].as<double>();
  params.g = constant["g"].as<double>();
  params.T = constant["T"].as<double>();
  params.F = constant["F"].as<double>();
  params.groundMu = constant["mu_ground"].as<double>();
  params.ballMu = constant["mu_ball"].as<double>();
  params.boxMu = constant["mu_box"].as<double>();
  params.initPenetration = constant["penentration0"].as<double>();

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
  ru::logger->addVariableToLog(3, "velbox", "linear velocity of box");
  ru::logger->addVariableToLog(3, "velball", "linear velocity of ball");
  ru::logger->addVariableToLog(3, "posbox", "position of box");
  ru::logger->addVariableToLog(3, "posball", "position of ball");

  // timer
  std::string timer = name + "timer";
  ru::timer->setLogPath(path);
  ru::timer->setLogFileName(timer);
}

} // benchmark::rolling

#endif //BENCHMARK_ROLLING_HPP

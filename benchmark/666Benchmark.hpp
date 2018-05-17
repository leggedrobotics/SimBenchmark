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

namespace benchmark::sixsixsix {

// data lists
std::vector<double> errorList;
std::vector<double> energyList;

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

  // restitution
  bool elasticCollision = false;   /// for elastic collision test
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

  double g = -9.81;
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
  xmlPath += "../res/mujoco/666/sphere" + std::to_string(ncubic) + ".xml";

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
  yamlPath += "./yaml/666.yaml";

  return yamlPath;
}

/**
 * get log file directory path of test result
 *
 * @param erp
 * @return log directory path in string
 */
std::string getLogFilepath() {
  std::string logPath(__FILE__);
  while (logPath.back() != '/')
    logPath.erase(logPath.size() - 1, 1);

  logPath += "../data/666/log.csv";
  return logPath;
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
      ("elastic", "if elastic collision")
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

  // elastic collision
  if(vm.count("elastic")) {
    options.elasticCollision = true;
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
  params.g = constant["g"].as<double>();

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

double computeMeanEnergyError(double E0) {
  std::vector<double> energyErrors;
  energyErrors.reserve(energyList.size());

  for(int i = 0; i < energyList.size(); i++) {
    energyErrors.push_back(pow(energyList[i] - E0, 2));
  }
  return std::accumulate( energyErrors.begin(), energyErrors.end(), 0.0) / energyErrors.size();
}

/**
 *
 * @param E0 initial energy
 */
void printError(double E0, double time) {
  RAIINFO(
      std::endl << "time       = " << time << std::endl
                << "pen. error = "
                << std::accumulate( errorList.begin(), errorList.end(), 0.0) / errorList.size() << std::endl)

  if(benchmark::sixsixsix::options.elasticCollision) {
    RAIINFO(
        std::endl << "E error    = "
                  << computeMeanEnergyError(E0) << std::endl
    )
  }
}

void printCSV(std::string filePath,
              std::string sim,
              std::string solver,
              double time,
              double E0) {
  std::ofstream myfile;
  myfile.open (filePath, std::ios_base::app);

  if(options.elasticCollision)
    myfile
        << sim << ","
        << solver << ","
        << options.erpYN << ","
        << options.elasticCollision << ","
        << options.dt << ","
        << std::accumulate( errorList.begin(), errorList.end(), 0.0) / errorList.size() << ","
        << computeMeanEnergyError(E0) << ","
        << time
        << std::endl;
  else
    myfile
        << sim << ","
        << solver << ","
        << options.erpYN << ","
        << options.elasticCollision << ","
        << options.dt << ","
        << std::accumulate( errorList.begin(), errorList.end(), 0.0) / errorList.size() << ","
        << "" << ","
        << time
        << std::endl;

  myfile.close();
}

} // benchmark::sixsixsix


#endif //BENCHMARK_THOUSANDBENCHMARK_HPP

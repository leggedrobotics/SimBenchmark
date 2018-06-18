//
// Created by kangd on 09.05.18.
//

#ifndef BENCHMARK_BUILDINGBENCHMARK_HPP
#define BENCHMARK_BUILDINGBENCHMARK_HPP

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

/**
 * Building test is for testing stability of contact solution.
 * The test focuses on:
 *
 * 1. The stability of contact simulation
 *
 * Please read docs for more details
 */

namespace po = boost::program_options;
namespace ru = rai::Utils;

namespace benchmark::building {

/**
 * options for building simulation
 */
struct Option: benchmark::Option {
  // erp
  bool erpYN = false;

  // time step
  double dt = 0.001;

  // simulation time
  double T = 600.0;

  // collapse
  bool collapse = false;
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

//  int raiMaxIter = 100;
//  double raiThreshold = 1e-7;

  // simulation parameters
  /// Remark! doesn't work for MUJOCO
  int numFloor = 6;
  int numBase = 4;
  /// (num of blocks) = (numFloor) x (numBase) + (numFloor) x (numWall x 2 + 1)

  const float shortLen = 0.05;
  const float longLen = 0.2;
  const float heightLen = 0.1;

  // gravity
  double g = -9.81;

};
Parameter params;

struct Data {
  void setN(int n) {
    Data::n = n;
    numContacts.reserve(n);
  }

  double computeMeanContacts() {
    return std::accumulate( numContacts.begin(), numContacts.end(), 0.0) / numContacts.size();
  }

  double time;
  int step;
  int n;
  std::vector<int> numContacts;   // the number of contact for each step
};

/**
 * get XML file path for Mujoco
 *
 * @return xml path in string
 */
std::string getMujocoXMLpath() {

  std::string xmlPath(__FILE__);
  while (xmlPath.back() != '/')
    xmlPath.erase(xmlPath.size() - 1, 1);
  xmlPath += "../res/benchmark/kapla-benchmark/mujoco/kapla.xml";

  return xmlPath;
}


/**
 * get URDF file for bullet multibody
 *
 * @return urdf path in string
 */
std::string getBulletPlanePath() {

  std::string ballpath(__FILE__);
  while (ballpath.back() != '/')
    ballpath.erase(ballpath.size() - 1, 1);
  ballpath += "../res/benchmark/kapla-benchmark/bullet/plane.urdf";

  return ballpath;
}

/**
 * get URDF file for bullet multibody
 *
 * @return urdf path in string
 */
std::string getBulletBasePath() {

  std::string planepath(__FILE__);
  while (planepath.back() != '/')
    planepath.erase(planepath.size() - 1, 1);
  planepath += "../res/benchmark/kapla-benchmark/bullet/base.urdf";

  return planepath;
}

/**
 * get URDF file for bullet multibody
 *
 * @return urdf path in string
 */
std::string getBulletWallPath() {

  std::string planepath(__FILE__);
  while (planepath.back() != '/')
    planepath.erase(planepath.size() - 1, 1);
  planepath += "../res/benchmark/kapla-benchmark/bullet/wall.urdf";

  return planepath;
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
  yamlPath += "./yaml/kapla.yaml";

  return yamlPath;
}

std::string getCSVpath() {

  std::string csvPath(__FILE__);
  while (csvPath.back() != '/')
    csvPath.erase(csvPath.size() - 1, 1);

  csvPath += "../data/kapla/" + options.csvName;

  return csvPath;
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

  dirPath += "../data/building/erp=" + std::to_string(erpYN)
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
      ("T", po::value<double>(), "simulation time (e.g. 60)")
      ("collapse", "stop simulation when the kapla tower is collapsed")
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

  // csv
  if(vm.count("csv")) {
    options.csv = true;
    options.csvName = vm["csv"].as<std::string>();
  }

  // collapse
  if(vm.count("collapse")) {
    options.collapse = true;
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
  params.numFloor = constant["num_floor"].as<int>();
  params.numBase = constant["num_base"].as<int>();

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
  std::string timer = name + "timer";
  ru::timer->setLogPath(path);
  ru::timer->setLogFileName(timer);
}

void printCSV(std::string filePath,
              std::string sim,
              std::string solver,
              std::string detector,
              std::string integrator,
              double time,
              double steps,
              double numContacts) {
  std::ofstream myfile;
  myfile.open (filePath, std::ios_base::app);

  myfile << sim << ","
         << solver << ","
         << detector << ","
         << integrator << ","
         << options.erpYN << ","
         << options.dt << ","
         << steps << ","
         << numContacts << ","
         << time << std::endl;
  myfile.close();
}

} // benchmark::rolling-benchmark


#endif //BENCHMARK_BUILDINGBENCHMARK_HPP

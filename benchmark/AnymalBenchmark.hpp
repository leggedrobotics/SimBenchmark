//
// Created by kangd on 26.04.18.
//

#ifndef BENCHMARK_ANYMAL_HPP
#define BENCHMARK_ANYMAL_HPP

#include <raiCommon/rai_utils.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

/**
 * ANYmal test (PD control) is articulated robot system test.
 * The test focuses on:
 *
 * 1. Speed of simulation
 * 2. Scalability
 *
 * Please read docs for more details
 */

namespace po = boost::program_options;

namespace benchmark::anymal {

/**
 * options for ANYmal simulation
 */
struct Option: benchmark::Option {
  // PD control to stand
  bool feedback = true;

  // # of robots = numRow x numRow
  int numRow = 1;
};
Option options;

/**
 * parameter for ANYmal simulation
 */
struct Parameter {

  // sim properties
  double lightPosition[3] = {30.0, 0, 10.0};

  // constans
  double kp = 40;     // kp gain
  double kd = 1.0;    // kd gain
  double H = 0.54;    // starting height
  double dt = 0.005;  // timestep (sec)
  double T = 250;      // simulation time (sec)
  double g = -9.81;

  // base quaternion
  double baseQuat[4] = {
      1.0, 0.0, 0.0, 0.0
  };

  // joint configuration
  double jointPos[12] = {
      0.03, 0.4, -0.8,
      -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8,
      -0.03, -0.4, 0.8
  };

  double dartjointPos[12] = {
      0.03, 0.4, -0.8,
      0.03, -0.4, 0.8,
      -0.03, 0.4, -0.8,
      -0.03, -0.4, 0.8
  };
};
Parameter params;

/**
 * get URDF file path of ANYmal
 *
 * @return urdfPath in string
 */
std::string getURDFpath() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/ANYmal-nomesh/robot.urdf";

  return urdfPath;
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
  yamlPath += "./yaml/anymal.yaml";

  return yamlPath;
}

/**
 * get URDF file path of ANYmal for Mujoco
 *
 * @param rowNum # of row
 * @return urdfPath in string
 */
std::string getMujocoURDFpath(int rowNum) {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/mujoco/ANYmal/robot" + std::to_string(rowNum * rowNum) + ".urdf";

  return urdfPath;
}

/**
 * get CSV log file path of test result
 *
 * @param feedback
 * @return log file path in string
 */
std::string getCSVpath(bool feedback) {

  std::string logPath(__FILE__);
  while (logPath.back() != '/')
    logPath.erase(logPath.size() - 1, 1);

  if(feedback)
    logPath += "../data/anymal-stand/" + options.csvName;
  else
    logPath += "../data/anymal-grounded/" + options.csvName;

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
      ("row", po::value<int>(), "the number of rows")
      ("feedback", po::value<bool>(), "feed back control y/n")
      ;
}

/**
 * get options from arguments
 *
 * @param argc
 * @param argv
 * @param desc
 */
void getOptionsFromArg(int argc, const char **argv, po::options_description &desc) {

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // help option
  if(vm.count("help")) {
    std::cout << desc << std::endl;
    exit(0);
  }

  // log option
  if(vm.count("csv")) {
    options.csv = true;
    options.csvName = vm["csv"].as<std::string>();
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

  // the number of row
  if(vm.count("row")) {
    options.numRow = vm["row"].as<int>();
  }

  // feed-back (PD control)
  if(vm.count("feedback")) {
    options.feedback = vm["feedback"].as<bool>();
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
  params.kp = constant["kp"].as<double>();
  params.kd = constant["kd"].as<double>();
  params.g = constant["g"].as<double>();
  params.H = constant["H"].as<double>();
  params.dt = constant["dt"].as<double>();
  params.T = constant["T"].as<double>();

  for(int i = 0; i < 12; i ++) {
    params.jointPos[i] = constant["jointPos"].as<std::vector<double>>()[i];
    params.dartjointPos[i] = constant["jointPosDart"].as<std::vector<double>>()[i];
  }

  params.baseQuat[0] = constant["baseQuat"].as<std::vector<double >>()[0];
  params.baseQuat[1] = constant["baseQuat"].as<std::vector<double >>()[1];
  params.baseQuat[2] = constant["baseQuat"].as<std::vector<double >>()[2];
  params.baseQuat[3] = constant["baseQuat"].as<std::vector<double >>()[3];

  // solver parameters
  YAML::Node solver_params = yaml["solver_params"];

  switch (simulator) {
    case benchmark::RAI:
      break;
    case benchmark::BULLET:
      break;
    case benchmark::ODE:
      break;
    case benchmark::MUJOCO:
      break;
    case benchmark::DART:
      break;
    default:
    RAIFATAL("invalid simulator value")
  }
}

void printCSV(std::string filePath,
              std::string sim,
              std::string solver,
              std::string detector,
              std::string integrator,
              int rownum,
              double time) {
  std::ofstream myfile;
  myfile.open (filePath, std::ios_base::app);
  myfile << sim << ","
         << solver << ","
         << detector << ","
         << integrator << ","
         << rownum << ","
         << time << std::endl;
  myfile.close();
}

} // benchmark::anymal

#endif //BENCHMARK_ANYMAL_HPP

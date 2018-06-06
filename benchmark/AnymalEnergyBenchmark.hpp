//
// Created by kangd on 14.05.18.
//

#ifndef BENCHMARK_ANYMALZEROGBENCHMARK_HPP
#define BENCHMARK_ANYMALZEROGBENCHMARK_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include "BenchmarkTest.hpp"

/**
 * ANYmal Energy test (free drop) checks energy conservation of articulated system model
 * The test focuses on:
 *
 * 1. Energy conservation vs simulation speed trade-off
 *
 * Please read docs for more details
 */

namespace po = boost::program_options;

namespace benchmark::anymal::freedrop {

/**
 * options for simulation
 */
struct Option: benchmark::Option {
  double dt = 0.005;  // timestep (sec)
  double guiRealtimeFactor = 0.2;
};
Option options;

/**
 * parameter for simulation
 */
struct Parameter {

  // sim properties
  double lightPosition[3] = {30.0, 0, 10.0};

  double T1 = 1;      // force applying time
  double T2 = 2;      // free drop time
  double H = 2;       // initial height
  double M = 0;       // will be updated! (anymal mass)
  double g = -9.81;
  double F = 0;       // will be updated
};
Parameter params;

/**
 * data for simulation
 */
struct Data {
  void setN(int n) {
    Data::n = n;
    kineticE.reserve(n);
    potentialE.reserve(n);
  }

  double computeError() {
    Eigen::MatrixXd energyErrorSq(n, 1);

    for(int i = 0; i < n; i++) {
      energyErrorSq(i, 0) = pow(kineticE[i] + potentialE[i] - E0, 2);
    }

    if(options.plot) {
      Eigen::MatrixXd tdata(n, 1);        // time
      Eigen::MatrixXd kEdata(n, 1);       // kinetic E
      Eigen::MatrixXd pEdata(n, 1);       // potential E
      Eigen::MatrixXd tEdata(n, 1);       // total E

      for(int i = 0; i < n; i++) {
        tdata(i, 0) = i * options.dt;
        kEdata(i, 0) = kineticE[i];
        pEdata(i, 0) = potentialE[i];
        tEdata(i, 0) = potentialE[i] + kineticE[i];
      }

      rai::Utils::Graph::FigProp2D figure1properties("time", "squared energy error", "squared energy error");
      rai::Utils::graph->figure(1, figure1properties);
      rai::Utils::graph->appendData(1, tdata.data(), energyErrorSq.data(), n, "error sq");
      rai::Utils::graph->drawFigure(1);

      rai::Utils::Graph::FigProp2D figure2properties("time", "momentum", "momentum");
      rai::Utils::graph->figure(2, figure2properties);
      rai::Utils::graph->appendData(2, tdata.data(), kEdata.data(), n, "kinetic E");
      rai::Utils::graph->appendData(2, tdata.data(), pEdata.data(), n, "potential E");
      rai::Utils::graph->appendData(2, tdata.data(), tEdata.data(), n, "total E");
      rai::Utils::graph->drawFigure(2);
      rai::Utils::graph->waitForEnter();
    }

    return energyErrorSq.mean();
  }

  std::vector<double> kineticE;
  std::vector<double> potentialE;

  // num data
  double E0 = 0;
  int n = 0;
};
Data data;

/**
 * get URDF file path of ANYmal
 *
 * @return urdfPath in string
 */
std::string getURDFpath() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/ANYmal-energy/robot.urdf";

  return urdfPath;
}

/**
 * get URDF file path of ANYmal
 *
 * @return urdfPath in string
 */
std::string getBulletPlanePath() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/bullet/ANYmal-energy/plane.urdf";

  return urdfPath;
}

/**
 * get URDF file path of ANYmal for Bullet
 *
 * @return urdfPath in string
 */
std::string getBulletANYmalPath() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/bullet/ANYmal-energy/robot.urdf";

  return urdfPath;
}

/**
 * get URDF file path of ANYmal for Mujoco
 *
 * @return urdfPath in string
 */
std::string getMujocoURDFpath() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/mujoco/ANYmal/robot-energy.urdf";

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
  yamlPath += "./yaml/anymal-energy.yaml";

  return yamlPath;
}

/**
 * get rlog file path of test result
 *
 * @param feedback
 * @return log file path in string
 */
std::string getLogDirpath(bool feedback) {

  std::string logPath(__FILE__);
  while (logPath.back() != '/')
    logPath.erase(logPath.size() - 1, 1);

  logPath += "../data/anymal-feedrop/";
  return logPath;
}

std::string getCSVpath() {

  std::string csvPath(__FILE__);
  while (csvPath.back() != '/')
    csvPath.erase(csvPath.size() - 1, 1);

  csvPath += "../data/anymal-energy/" + options.csvName;

  return csvPath;
}

/**
 * add options to desc
 *
 * @param desc
 */
void addDescToOption(po::options_description &desc) {
  benchmark::addDescToOption(desc);

  desc.add_options()
      ("dt", po::value<double>(), "time step for simulation (e.g. 0.01)")
      ("plot", "plot energy error")
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

  // dt
  if(vm.count("dt")) {
    options.dt = vm["dt"].as<double>();
  }

  // csv
  if(vm.count("csv")) {
    options.csv = true;
    options.csvName = vm["csv"].as<std::string>();
  }

  // plot
  if(vm.count("plot")) {
    options.plot = true;
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
  params.g = constant["g"].as<double>();
  params.H = constant["H"].as<double>();
  params.T1 = constant["T1"].as<double>();
  params.T2 = constant["T2"].as<double>();

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
              double time,
              double error) {
  std::ofstream myfile;
  myfile.open (filePath, std::ios_base::app);
  myfile << sim << ","
         << solver << ","
         << detector << ","
         << integrator << ","
         << options.dt << ","
         << error << ","
         << time << std::endl;
  myfile.close();
}

} // benchmark::anymal

#endif //BENCHMARK_ANYMALZEROGBENCHMARK_HPP

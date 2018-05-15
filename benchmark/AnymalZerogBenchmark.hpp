//
// Created by kangd on 14.05.18.
//

#ifndef BENCHMARK_ANYMALZEROGBENCHMARK_HPP
#define BENCHMARK_ANYMALZEROGBENCHMARK_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

/**
 * ANYmal zero g test is ...
 * The test focuses on:
 *
 * 1. Momentum conservation
 * 2. Speed of simulation
 *
 * Please read docs for more details
 */

namespace po = boost::program_options;

namespace benchmark::anymal::zerogravity {

// data lists
std::vector<double> errorList;

/**
 * options for ANYmal simulation
 */
struct Option: benchmark::Option {
  double dt = 0.005;  // timestep (sec)
};
Option options;

/**
 * parameter for ANYmal simulation
 */
struct Parameter {
  double T = 10;      // simulation time (sec)
  double H = 2;
  double x0 = -5;
  double v0 = 2;
  double m = 0;       // will be updated!
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
  urdfPath += "../res/ANYmal-nomesh/";

  return urdfPath;
}

/**
 * get URDF file path of ANYmal for Mujoco
 *
 * @param rowNum # of row
 * @return urdfPath in string
 */
std::string getMujocoURDFpath() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/mujoco/ANYmal/robot-momentum.urdf";

  return urdfPath;
}

/**
 * get CSV log file path of test result
 *
 * @param feedback
 * @return log file path in string
 */
std::string getLogFilepath(bool feedback) {

  std::string logPath(__FILE__);
  while (logPath.back() != '/')
    logPath.erase(logPath.size() - 1, 1);

  logPath += "../data/anymal-zero-g/log.csv";
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
      ("dt", po::value<double>(), "time step for simulation (e.g. 0.01)")
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
}

void printCSV(std::string filePath,
              std::string sim,
              std::string solver,
              std::string detector,
              int rownum,
              double time) {
  std::ofstream myfile;
  myfile.open (filePath, std::ios_base::app);
  myfile << sim << "," << solver << "," << detector << "," << rownum << "," << time << std::endl;
  myfile.close();
}

} // benchmark::anymal

#endif //BENCHMARK_ANYMALZEROGBENCHMARK_HPP

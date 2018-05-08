//
// Created by kangd on 26.04.18.
//

#ifndef BENCHMARK_ANYMAL_HPP
#define BENCHMARK_ANYMAL_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

/**
 * ANYmal test is for testing articulated robot system.
 * The test focuses on:
 *
 * 1. Speed of simulation
 * 2. Scalability
 *
 * Please read benchmark/README.md for more details
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
  double kp = 40;     // kp gain
  double kd = 1.0;    // kd gain
  double H = 0.54;    // starting height
  double dt = 0.005;  // timestep (sec)
  double T = 250;      // simulation time (sec)

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
  urdfPath += "../res/ANYmal-nomesh/";

  return urdfPath;
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
std::string getLogFilepath(bool feedback) {

  std::string logPath(__FILE__);
  while (logPath.back() != '/')
    logPath.erase(logPath.size() - 1, 1);

  if(feedback)
    logPath += "../data/anymal-stand/log.csv";
  else
    logPath += "../data/anymal-grounded/log.csv";

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

  // the number of row
  if(vm.count("row")) {
    options.numRow = vm["row"].as<int>();
  }

  // feed-back (PD control)
  if(vm.count("feedback")) {
    options.feedback = vm["feedback"].as<bool>();
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

#endif //BENCHMARK_ANYMAL_HPP

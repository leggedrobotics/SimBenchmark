//
// Created by kangd on 26.04.18.
//

#ifndef BENCHMARK_ANYMAL_HPP
#define BENCHMARK_ANYMAL_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace benchmark::anymal {

/**
 * options for ANYmal simulation
 */
struct Option {
  bool gui = true;
  bool feedback = true;
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

std::string getMujocoURDFpath(int rowNum) {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/mujoco/ANYmal/robot" + std::to_string(rowNum * rowNum) + ".urdf";

  return urdfPath;
}

/**
 * add options to desc
 *
 * @param desc
 */
void addDescToOption(po::options_description &desc) {
  desc.add_options()
      ("help", "produce help message")
      ("nogui", "no visualization")
      ("row", po::value<int>(), "the number of rows")
      ("feedback", po::value<bool>(), "feed back control y/n")
      ;
}

/**
 * get option or parameter from arguments
 *
 * @param argc
 * @param argv
 * @param desc
 */
void getParamsFromArg(int argc, const char *argv[], po::options_description &desc) {

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // help option
  if(vm.count("help")) {
    std::cout << desc << std::endl;
    exit(0);
  }

  // nogui option
  if(vm.count("nogui")) {
    options.gui = false;
  }

  // the number of row
  if(vm.count("row")) {
    options.numRow = vm["row"].as<int>();
  }

  // feed back erp
  if(vm.count("feedback")) {
    options.feedback = vm["feedback"].as<bool>();
  }
}

} // benchmark::anymal

#endif //BENCHMARK_ANYMAL_HPP

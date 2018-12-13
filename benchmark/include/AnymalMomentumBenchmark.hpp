//
// Created by kangd on 14.05.18.
//

#ifndef BENCHMARK_ANYMALMOMENTUMBENCHMARK_HPP
#define BENCHMARK_ANYMALMOMENTUMBENCHMARK_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include "BenchmarkTest.hpp"

/**
 * ANYmal momentum test (zero gravity) investigates momentum conservation of articulated system collision.
 * The test focuses on:
 *
 * 1. Momentum conservation vs simulation speed trade-off
 *
 * Please read docs for more details
 */

namespace po = boost::program_options;

namespace benchmark::anymal::zerogravity {


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
        // sim properties
        double lightPosition[3] = {30.0, 0, 10.0};

        double T = 10;      // simulation time (sec)
        double H = 2.0;
        double x0 = -5;
        double v0 = 5;
        double M = 0;       // will be updated!
        double m = 10;
        double g = -9.81;
    };
    Parameter params;

    struct Data {
        void setN(int n) {
          Data::n = n;
          ballMomentum.reserve(n);
          anymalMomentum.reserve(n);
        }

        double computeError() {
          Eigen::MatrixXd momentumErrorSq(n, 1);
          Eigen::Vector3d momentum = {0, params.m * params.v0, 0};

          for(int i = 0; i < n; i++) {
            momentumErrorSq(i, 0) = pow((ballMomentum[i] + anymalMomentum[i] - momentum).norm(),2);
          }

          if(options.plot) {
            Eigen::MatrixXd tdata(n, 1);        // time
            Eigen::MatrixXd bMdata(n, 1);        // ball momentum y
            Eigen::MatrixXd aMdata(n, 1);        // anymal momentum y
            Eigen::MatrixXd tMdata(n, 1);        // total momentum y

            for(int i = 0; i < n; i++) {
              tdata(i, 0) = i * options.dt;
              bMdata(i, 0) = ballMomentum[i].y();
              aMdata(i, 0) = anymalMomentum[i].y();
              tMdata(i, 0) = (ballMomentum[i] + anymalMomentum[i]).y();
            }

            rai::Utils::Graph::FigProp2D figure1properties("time", "squared momentum error", "squared momentum error");
            rai::Utils::graph->figure(1, figure1properties);
            rai::Utils::graph->appendData(1, tdata.data(), momentumErrorSq.data(), n, "error norm sq");
            rai::Utils::graph->drawFigure(1);

            rai::Utils::Graph::FigProp2D figure2properties("time", "momentum", "momentum");
            rai::Utils::graph->figure(2, figure2properties);
            rai::Utils::graph->appendData(2, tdata.data(), bMdata.data(), n, "ball momentum");
            rai::Utils::graph->appendData(2, tdata.data(), aMdata.data(), n, "anymal momentum");
            rai::Utils::graph->appendData(2, tdata.data(), tMdata.data(), n, "total momentum");
            rai::Utils::graph->drawFigure(2);
            rai::Utils::graph->waitForEnter();
          }

          return momentumErrorSq.mean();
        }

        // data list
        std::vector<Eigen::Vector3d> ballMomentum;
        std::vector<Eigen::Vector3d> anymalMomentum;

        // num data
        int n = 0;
    };
    Data data;

/**
 * get URDF file path of ANYmal
 *
 * @return urdfPath in string
 */
    std::string getBulletPlanePath() {

      std::string urdfPath(RESOURCEPATH);
      urdfPath += "/benchmark/ANYmal-momentum-benchmark/bullet/plane.urdf";

      return urdfPath;
    }

/**
 * get URDF file path of ANYmal
 *
 * @return urdfPath in string
 */
    std::string getBulletBallPath() {

      std::string urdfPath(RESOURCEPATH);
      urdfPath += "/benchmark/ANYmal-momentum-benchmark/bullet/ball.urdf";

      return urdfPath;
    }

/**
 * get URDF file path of ANYmal
 *
 * @return urdfPath in string
 */
    std::string getBulletANYmalPath() {

      std::string urdfPath(RESOURCEPATH);
      urdfPath += "/benchmark/ANYmal-momentum-benchmark/bullet/robot.urdf";

      return urdfPath;
    }

/**
 * get URDF file path of ANYmal
 *
 * @return urdfPath in string
 */
    std::string getURDFpath() {

      std::string urdfPath(RESOURCEPATH);
      urdfPath += "/benchmark/ANYmal-momentum-benchmark/ode-rai-dart/robot.urdf";

      return urdfPath;
    }

/**
 * get URDF file path of ANYmal for Mujoco
 *
 * @param rowNum # of row
 * @return urdfPath in string
 */
    std::string getMujocoURDFpath() {

      std::string urdfPath(RESOURCEPATH);
      urdfPath += "/benchmark/ANYmal-momentum-benchmark/mujoco/robot.urdf";

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

      logPath += "../data/anymal-momentum/log.csv";
      return logPath;
    }

    std::string getCSVpath() {

      std::string csvPath(__FILE__);
      while (csvPath.back() != '/')
        csvPath.erase(csvPath.size() - 1, 1);

      csvPath += "../data/anymal-momentum/" + options.csvName;

      return csvPath;
    }

/**
 * get YAML file path for parameter
 *
 * @return yaml path in string
 */
    std::string getYamlpath() {

      std::string yamlPath(BENCHMARKYAMLPATH);
      yamlPath += "/anymal-momentum.yaml";

      return yamlPath;
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
              ("plot", "plot momentum error")
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
      params.T = constant["T"].as<double>();
      params.x0 = constant["x0"].as<double>();
      params.v0 = constant["v0"].as<double>();
      params.m = constant["m"].as<double>();

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

#endif //BENCHMARK_ANYMALMOMENTUMBENCHMARK_HPP

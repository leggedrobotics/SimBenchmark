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

/// functions
    std::string getCSVpath();
    std::string getMujocoXMLpath();
    std::string getBulletBallPath();
    std::string getBulletPlanepath();
    std::string getYamlPath();
    std::string getLogDirpath(bool erpYN, double restCoeff, std::string simulation, std::string solver, double dt);
    void addDescToOption(po::options_description &desc);
    void getOptionsFromArg(int argc, const char *argv[], po::options_description &desc);
    void getParamsFromYAML(const char *yamlfile, benchmark::Simulator simulator);
    void loggerSetup(std::string path, std::string name);
    void printCSV(std::string, std::string, std::string, std::string, std::string, double, double error);

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

        // friction option
        bool friction = false;
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
        double m = 10;              // mass of ball
        int n = 7;                 // (num obj) = n x n
        double H = 5;
        double R = 0.1;               // radius of ball
        double g = -9.81;
        double T = 20;

        // only when friction is on (if not both are zero)
        double mu_ground = 0;
        double mu_ball = 0;

        /// note
        /// 1. (frictional coeff A-B) = (friction coeff of A) x (friction coeff of B)             - Bullet & ODE
        /// 2. (frictional coeff A-B) = max of (friction coeff of A) and (friction coeff of B)    - Mujoco
        /// 3. (frictional coeff A-B) = min of (friction coeff of A) and (friction coeff of B)    - Dart
        /// 4. (frictional coeff A-B)                                                             - Rai
    };
    Parameter params;

    struct Data {
        void setN(int n) {
          Data::n = n;
          ballEnergy.reserve(n);
        }

        double computeError() {
          Eigen::MatrixXd energyErrorSq(n, 1);

          double E =
                  - benchmark::bouncing::params.m * benchmark::bouncing::params.n * benchmark::bouncing::params.n
                  * benchmark::bouncing::params.g * benchmark::bouncing::params.H;

          for(int i = 0; i < n; i++) {
            energyErrorSq(i, 0) = pow(ballEnergy[i] - E,2);
          }

          if(options.plot) {
            Eigen::MatrixXd tdata(n, 1);        // time

            for(int i = 0; i < n; i++) {
              tdata(i, 0) = i * benchmark::bouncing::options.dt;
            }

            rai::Utils::Graph::FigProp2D figure1properties("time", "squared energy error", "squared energy error");
            rai::Utils::graph->figure(1, figure1properties);
            rai::Utils::graph->appendData(1, tdata.data(), energyErrorSq.data(), n, "E error sq");
            rai::Utils::graph->drawFigure(1);
            rai::Utils::graph->waitForEnter();
          }

          return energyErrorSq.mean();
        }

        // data list
        std::vector<double> ballEnergy;

        // num data
        int n = 0;
    };
    Data data;

    std::string getCSVpath() {

      std::string csvPath(__FILE__);
      while (csvPath.back() != '/')
        csvPath.erase(csvPath.size() - 1, 1);

      csvPath += "../data/bouncing/" + options.csvName;

      return csvPath;
    }

/**
 * get XML file path for Mujoco
 *
 * @return urdf path in string
 */
    std::string getMujocoXMLpath() {

      const int nsq = (params.n) * (params.n);
      std::string xmlPath(RESOURCEPATH);
      xmlPath += "/benchmark/bouncing-benchmark/mujoco/bouncing" + std::to_string(nsq) + ".xml";

      return xmlPath;
    }

/**
 * get XML file path for Mujoco
 *
 * @return urdf path in string
 */
    std::string getBulletBallPath() {

      std::string ballpath(RESOURCEPATH);
      ballpath += "/benchmark/bouncing-benchmark/bullet/ball.urdf";

      return ballpath;
    }

/**
 * get XML file path for Mujoco
 *
 * @return urdf path in string
 */
    std::string getBulletPlanepath() {

      std::string planepath(RESOURCEPATH);
      planepath += "/benchmark/bouncing-benchmark/bullet/plane.urdf";

      return planepath;
    }

/**
 * get YAML file path for parameter
 *
 * @return yaml path in string
 */
    std::string getYamlPath() {

      std::string yamlPath(BENCHMARKYAMLPATH);
      yamlPath += "/bouncing.yaml";

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
              ("friction", "non-zero friction")
              ("plot", "plot on")
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
        RAIWARN_IF(options.e != 1, "only elastic collision analytic solution is supported")
      }

      // plot option
      if(vm.count("plot")) {
        options.plot = true;
      }

      // erp
      if(vm.count("erp-on")) {
        if(vm["erp-on"].as<bool>()) {
          options.erpYN = true;
        } else {
          options.erpYN = false;
        }
      }

      // friction
      if(vm.count("friction")) {
        options.friction = true;
      }

      // csv
      if(vm.count("csv")) {
        options.csv = true;
        options.csvName = vm["csv"].as<std::string>();
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
      params.H = constant["H"].as<double>();

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
                  double error) {
      std::ofstream myfile;
      myfile.open (filePath, std::ios_base::app);

      myfile << sim << ","
             << solver << ","
             << detector << ","
             << integrator << ","
             << options.erpYN << ","
             << options.e << ","
             << options.dt << ","
             << error << ","
             << time << std::endl;
      myfile.close();
    }

} // benchmark::bouncing

#endif //BENCHMARK_BOUNCINGBENCHMARK_HPP

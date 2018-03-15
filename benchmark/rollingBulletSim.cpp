//
// Created by kangd on 15.02.18.
//

#include <bulletSim/World_RG.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "rolling.hpp"

namespace po = boost::program_options;
namespace ru = rai::Utils;

// sim
bullet_sim::World_RG *sim;

// functions
void getParams(int argc, const char* argv[], char* yamlfile);
void simulationSetup();
void loggerSetup();

// variables
benchmark::Params params;
benchmark::Options options;
bullet_sim::SolverOption solverOption = bullet_sim::SOLVER_SEQUENTIAL_IMPULSE;

std::vector<benchmark::SingleBodyHandle> objectList;

int main(int argc, const char* argv[]) {

  // get parameter from argument and yaml
  getParams(argc, argv, "./rolling.yaml");

  // set up logger and timer
  loggerSetup();

  // set up simulation
  simulationSetup();

  // simulation loop
  // press 'q' key to quit
  ru::timer->startTimer("rolling");
  if(options.visualize) {
    for(int i = 0; i < params.T / params.dt && sim->visualizerLoop(params.dt); i++) {
      if(options.forceDirection == benchmark::FORCE_Y)
        objectList[0]->setExternalForce(Eigen::Vector3d(0, params.F, 0));
      else if(options.forceDirection == benchmark::FORCE_XY)
        objectList[0]->setExternalForce(Eigen::Vector3d(params.F * 0.707106781186547,
                                              params.F * 0.707106781186547, 0));

      // log
      ru::logger->appendData("velbox", objectList[0]->getLinearVelocity().data());
      ru::logger->appendData("velball", objectList[1]->getLinearVelocity().data());
      ru::logger->appendData("posbox", objectList[0]->getPosition().data());
      ru::logger->appendData("posball", objectList[1]->getPosition().data());
      sim->integrate(params.dt);
    }
  }
  else {
    for(int i = 0; i < params.T / params.dt; i++) {
      if(options.forceDirection == benchmark::FORCE_Y)
        objectList[0]->setExternalForce(Eigen::Vector3d(0, params.F, 0));
      else if(options.forceDirection == benchmark::FORCE_XY)
        objectList[0]->setExternalForce(Eigen::Vector3d(params.F * 0.707106781186547,
                                              params.F * 0.707106781186547, 0));

      // log
      ru::logger->appendData("velbox", objectList[0]->getLinearVelocity().data());
      ru::logger->appendData("velball", objectList[1]->getLinearVelocity().data());
      ru::logger->appendData("posbox", objectList[0]->getPosition().data());
      ru::logger->appendData("posball", objectList[1]->getPosition().data());
      sim->integrate(params.dt);
    }
  }
  ru::timer->stopTimer("rolling");

  // delete sim
  delete sim;

  // time log
  ru::timer->dumpToStdOuput();

  return 0;
}

void getParams(int argc, const char *argv[], char *yamlfile) {

  /// parameters from yaml
  YAML::Node yaml = YAML::LoadFile(yamlfile);

  // simulation options
  params.dt = yaml["options"]["dt"].as<double>();
  if(yaml["options"]["force_direction"].as<std::string>().compare("xy") == 0) {
    options.forceDirection = benchmark::FORCE_XY;
  } else if(yaml["options"]["force_direction"].as<std::string>().compare("y") == 0) {
    options.forceDirection = benchmark::FORCE_Y;
  } else {
    RAIFATAL("invalid force-direction");
  }
  options.erpYN = yaml["options"]["erp_on"].as<bool>();

  // solver parameters
  params.erp = yaml["solver_params"]["raiSim"]["erp"].as<double>();

  // sim properties
  YAML::Node props = yaml["sim_properties"];
  params.lightPosition = props["light_position"].as<std::vector<double>>();

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

  /// parameter from arguments
  po::options_description desc("Allowed options");
  desc.add_options()
      ("nogui", "no visualization")
      ("noerp", "no erp")
      ("solver", po::value<std::string>(), "contact solver (seqImp / nngc / pgs / dantzig / lemke)")
      ("dt", po::value<double>(), "time step for simulation")
      ("force-direction", po::value<std::string>(), "applied force direction (y / xy)")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // help option
  if(vm.count("help")) {
    RAIINFO(desc)
  }

  // nogui option
  if(vm.count("nogui")) {
    options.visualize = false;
  }

  // solver option
  if(vm.count("solver")) {
    std::string solverStr = vm["solver"].as<std::string>();

    if(solverStr.compare("seqImp")==0) {
      solverOption = bullet_sim::SOLVER_SEQUENTIAL_IMPULSE;
      options.solverName = "seqImp";
    } else if(solverStr.compare("nncg")==0) {
      solverOption = bullet_sim::SOLVER_NNCG;
      options.solverName = "nncg";
    } else if(solverStr.compare("pgs")==0) {
      solverOption = bullet_sim::SOLVER_MLCP_PGS;
      options.solverName = "pgs";
    } else if(solverStr.compare("dantzig")==0) {
      solverOption = bullet_sim::SOLVER_MLCP_DANTZIG;
      options.solverName = "dantzig";
    } else if(solverStr.compare("lemke")==0) {
      solverOption = bullet_sim::SOLVER_MLCP_LEMKE;
      options.solverName = "lemke";
    } else {
      RAIFATAL("invalid solver option")
    }
  } else {
    options.solverName = "seqImp";
  }

  // dt option
  if(vm.count("dt")) {
    params.dt = vm["dt"].as<double>();
  }

  // force direction
  if(vm.count("force-direction")) {
    if(vm["force-direction"].as<std::string>().compare("xy") == 0) {
      options.forceDirection = benchmark::FORCE_XY;
    } else if(vm["force-direction"].as<std::string>().compare("y") == 0) {
      options.forceDirection = benchmark::FORCE_Y;
    } else {
      RAIFATAL("invalid force-direction")
    }
  }

  // erp
  if(vm.count("noerp")) {
    options.erpYN = false;
  }

  RAIINFO("----------------------")
  RAIINFO("BulletSim")
  RAIINFO("timestep        = " << params.dt);
  RAIINFO("solver          = " << options.solverName);
  RAIINFO("erp             = " << options.erpYN);
  RAIINFO("force-direction = " << options.forceDirection);
}

void simulationSetup() {

  if(options.visualize)
    sim = new bullet_sim::World_RG(800, 600, 0.5, benchmark::NO_BACKGROUND, solverOption);
  else
    sim = new bullet_sim::World_RG(solverOption);

  sim->setGravity(Eigen::Vector3d(0, 0, params.g));
  if(options.erpYN)
    sim->setERP(params.erp, params.erp, params.erp);

  // add ground
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setFrictionCoefficient(params.groundMu);

  // visualization settings
  if(options.visualize) {
    // camera relative position
    sim->setLightPosition(params.lightPosition[0],
                          params.lightPosition[1],
                          params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {30, 0, 15});
  }

  // add objects
  auto box = sim->addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5 - params.initPenetration);
  box->setFrictionCoefficient(params.boxMu);
  objectList.push_back(box);

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5 - params.initPenetration * 2);
      ball->setFrictionCoefficient(params.ballMu);
      objectList.push_back(ball);
    }
  }
}

void loggerSetup() {
  // logger
  std::string path = benchmark::dataPath + options.parentDir + "bullet/" + options.solverName;
  std::string name = std::to_string(params.dt);
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
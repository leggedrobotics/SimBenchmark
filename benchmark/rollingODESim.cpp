//
// Created by kangd on 15.02.18.
//

#include "odeSim/World_RG.hpp"

#include "rolling.hpp"

namespace rb = rolling_benchmark;

// sim
ode_sim::World_RG *sim;

// functions
void getParams(int argc, const char* argv[], char* yamlfile);
void simulationSetup();

// variables
ode_sim::SolverOption solverOption = ode_sim::SOLVER_STANDARD;
std::vector<benchmark::SingleBodyHandle> objectList;

int main(int argc, const char* argv[]) {

  // get parameter from argument and yaml
  getParams(argc, argv, "./rolling.yaml");

  // set up logger and timer
  std::string parentDir =
      rb::parentDir + "-" +
          "erp" + "=" + std::to_string(rb::options.erpYN) + "-" +
          "dir" + "=" + std::to_string(rb::options.forceDirection) + "/";
  rb::loggerSetup(
      benchmark::dataPath + parentDir + "ode/" + rb::options.solverName,
      std::to_string(rb::params.dt));

  // set up simulation
  simulationSetup();

  // simulation loop
  // press 'q' key to quit
  ru::timer->startTimer("rolling");
  if(rb::options.visualize) {
    for(int i = 0; i < rb::params.T / rb::params.dt && sim->visualizerLoop(rb::params.dt); i++) {
      if(rb::options.forceDirection == rb::FORCE_Y)
        objectList[0]->setExternalForce(Eigen::Vector3d(0, rb::params.F, 0));
      else if(rb::options.forceDirection == rb::FORCE_XY)
        objectList[0]->setExternalForce(Eigen::Vector3d(rb::params.F * 0.707106781186547,
                                                        rb::params.F * 0.707106781186547, 0));

      // log
      ru::logger->appendData("velbox", objectList[0]->getLinearVelocity().data());
      ru::logger->appendData("velball", objectList[1]->getLinearVelocity().data());
      ru::logger->appendData("posbox", objectList[0]->getPosition().data());
      ru::logger->appendData("posball", objectList[1]->getPosition().data());
      sim->integrate(rb::params.dt);
    }
  }
  else {
    for(int i = 0; i < rb::params.T / rb::params.dt; i++) {
      if(rb::options.forceDirection == rb::FORCE_Y)
        objectList[0]->setExternalForce(Eigen::Vector3d(0, rb::params.F, 0));
      else if(rb::options.forceDirection == rb::FORCE_XY)
        objectList[0]->setExternalForce(Eigen::Vector3d(rb::params.F * 0.707106781186547,
                                                        rb::params.F * 0.707106781186547, 0));

      // log
      ru::logger->appendData("velbox", objectList[0]->getLinearVelocity().data());
      ru::logger->appendData("velball", objectList[1]->getLinearVelocity().data());
      ru::logger->appendData("posbox", objectList[0]->getPosition().data());
      ru::logger->appendData("posball", objectList[1]->getPosition().data());
      sim->integrate(rb::params.dt);
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

  // sim specific
  rb::params.erp = yaml["solver_params"]["ode"]["erp"].as<double>();

  // generic
  rb::getParamsFromYAML(yamlfile);

  /// parameter from arguments
  // sim specific
  po::options_description simdesc("sim specific");
  simdesc.add_options()
      ("solver", po::value<std::string>(), "contact solver (std / quick)")
      ;
  rb::desc.add(simdesc);

  // generic
  rb::getParamsFromArg(argc, argv);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, rb::desc), vm);

  // solver option
  if(vm.count("solver")) {
    std::string solverStr = vm["solver"].as<std::string>();

    if(solverStr.compare("std")==0) {
      solverOption = ode_sim::SOLVER_STANDARD;
      rb::options.solverName = "std";
    } else if(solverStr.compare("quick")==0) {
      solverOption = ode_sim::SOLVER_QUICK;
      rb::options.solverName = "quick";
    } else {
      RAIFATAL("invalid solver option")
    }
  } else {
    rb::options.solverName = "std";
  }

  RAIINFO("----------------------")
  RAIINFO("odeSim")
  RAIINFO("timestep        = " << rb::params.dt);
  RAIINFO("solver          = " << rb::options.solverName);
  RAIINFO("erpYN           = " << rb::options.erpYN);
  RAIINFO("force-direction = " << rb::options.forceDirection);
}

void simulationSetup() {

  if(rb::options.visualize)
    sim = new ode_sim::World_RG(800, 600, 0.5, benchmark::NO_BACKGROUND, solverOption);
  else
    sim = new ode_sim::World_RG(solverOption);

  sim->setGravity(Eigen::Vector3d(0, 0, rb::params.g));
  if(rb::options.erpYN)
    sim->setERP(rb::params.erp, 0, 0);

  // add ground
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setFrictionCoefficient(rb::params.groundMu);

  // visualization settings
  if(rb::options.visualize) {
    // camera relative position
    sim->setLightPosition(rb::params.lightPosition[0],
                          rb::params.lightPosition[1],
                          rb::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {30, 0, 15});
  }

  // add objects
  auto box = sim->addBox(20, 20, 1, rb::params.M);
  box->setPosition(0, 0, 0.5 - rb::params.initPenetration);
  box->setFrictionCoefficient(rb::params.boxMu);
  objectList.push_back(box);

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim->addSphere(0.5, rb::params.m);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5 - rb::params.initPenetration * 3);
      ball->setFrictionCoefficient(rb::params.ballMu);
      objectList.push_back(ball);
    }
  }
}
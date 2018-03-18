//
// Created by kangd on 21.02.18.
//

#include <mujocoSim/World_RG.hpp>

#include "rolling.hpp"

namespace rb = rolling_benchmark;

// sim
mujoco_sim::World_RG *sim;

// functions
void getParams(int argc, const char* argv[], char* yamlfile);
void simulationSetup(const char *modelPath, const char *keyPath);

// variables
mujoco_sim::SolverOption solverOption = mujoco_sim::SOLVER_PGS;

int main(int argc, const char* argv[]) {

  // get parameter from argument and yaml
  getParams(argc, argv, "./rolling.yaml");

  // set up logger and timer
  std::string parentDir =
      rb::parentDir + "-" +
          "erp" + "=" + std::to_string(rb::options.erpYN) + "-" +
          "dir" + "=" + std::to_string(rb::options.forceDirection) + "/";
  rb::loggerSetup(
      benchmark::dataPath + parentDir + "mujoco/" + rb::options.solverName,
      std::to_string(rb::params.dt));

  // set up simulation
  simulationSetup("./mujoco/rolling.xml", "../mjkey.txt");

  // simulation loop
  // press 'q' key to quit
  rai::Utils::timer->startTimer("rolling");
  if(rb::options.visualize) {
    for(int i = 0; i < rb::params.T / rb::params.dt && sim->visualizerLoop(rb::params.dt); i++) {
      if(rb::options.forceDirection == rolling_benchmark::FORCE_Y)
        sim->getSingleBodyHandle(1)->setExternalForce(Eigen::Vector3d(0, rb::params.F, 0));
      else if(rb::options.forceDirection == rolling_benchmark::FORCE_XY)
        sim->getSingleBodyHandle(1)->setExternalForce(Eigen::Vector3d(rb::params.F * 0.707106781186547,
                                                        rb::params.F * 0.707106781186547,
                                                        0));

      // log
      rai::Utils::logger->appendData("velbox", sim->getSingleBodyHandle(1)->getLinearVelocity().data());
      rai::Utils::logger->appendData("velball", sim->getSingleBodyHandle(2)->getLinearVelocity().data());
      rai::Utils::logger->appendData("posbox", sim->getSingleBodyHandle(1)->getPosition().data());
      rai::Utils::logger->appendData("posball", sim->getSingleBodyHandle(2)->getPosition().data());
      sim->integrate(rb::params.dt);
    }
  }
  else {
    for(int i = 0; i < rb::params.T / rb::params.dt; i++) {
      if(rb::options.forceDirection == rolling_benchmark::FORCE_Y)
        sim->getSingleBodyHandle(1)->setExternalForce(Eigen::Vector3d(0, rb::params.F, 0));
      else if(rb::options.forceDirection == rolling_benchmark::FORCE_XY)
        sim->getSingleBodyHandle(1)->setExternalForce(Eigen::Vector3d(rb::params.F * 0.707106781186547,
                                                        rb::params.F * 0.707106781186547, 0));

      // log
      rai::Utils::logger->appendData("velbox", sim->getSingleBodyHandle(1)->getLinearVelocity().data());
      rai::Utils::logger->appendData("velball", sim->getSingleBodyHandle(2)->getLinearVelocity().data());
      rai::Utils::logger->appendData("posbox", sim->getSingleBodyHandle(1)->getPosition().data());
      rai::Utils::logger->appendData("posball", sim->getSingleBodyHandle(2)->getPosition().data());
      sim->integrate(rb::params.dt);
    }
  }
  rai::Utils::timer->stopTimer("rolling");

  // delete sim
  delete sim;

  // time log
  rai::Utils::timer->dumpToStdOuput();

  return 0;
}

void getParams(int argc, const char *argv[], char *yamlfile) {

  /// parameters from yaml
  YAML::Node yaml = YAML::LoadFile(yamlfile);

  // generic
  rb::getParamsFromYAML(yamlfile);

  /// parameter from arguments
  // sim specific
  po::options_description simdesc("sim specific");
  simdesc.add_options()
      ("solver", po::value<std::string>(), "contact solver (pgs / cg / newton)")
      ;
  rb::desc.add(simdesc);

  // generic
  rb::getParamsFromArg(argc, argv);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, rb::desc), vm);

  // solver option
  if(vm.count("solver")) {
    std::string solverStr = vm["solver"].as<std::string>();

    if(solverStr.compare("pgs")==0) {
      solverOption = mujoco_sim::SOLVER_PGS;
      rb::options.solverName = "pgs";
    } else if(solverStr.compare("cg")==0) {
      solverOption = mujoco_sim::SOLVER_CG;
      rb::options.solverName = "cg";
    } else if(solverStr.compare("newton")==0) {
      solverOption = mujoco_sim::SOLVER_NEWTON;
      rb::options.solverName = "newton";
    } else {
      RAIFATAL("invalid solver option")
    }
  } else {
    rb::options.solverName = "pgs";
  }

  RAIINFO("----------------------")
  RAIINFO("mujocoSim")
  RAIINFO("timestep        = " << rb::params.dt);
  RAIINFO("solver          = " << rb::options.solverName);
  RAIINFO("erpYN           = " << rb::options.erpYN);
  RAIINFO("force-direction = " << rb::options.forceDirection);
}

void simulationSetup(const char *modelPath, const char *keyPath) {

  if(rb::options.visualize)
    sim = new mujoco_sim::World_RG(800, 600, 0.5, modelPath, keyPath, benchmark::NO_BACKGROUND, solverOption);
  else
    sim = new mujoco_sim::World_RG(modelPath, keyPath, solverOption);

  // visualization settings
  if(rb::options.visualize) {
    // camera relative position
    sim->setLightPosition(rb::params.lightPosition[0],
                          rb::params.lightPosition[1],
                          rb::params.lightPosition[2]);
    sim->cameraFollowObject(sim->getSingleBodyHandle(0), {30, 0, 15});
  }
}
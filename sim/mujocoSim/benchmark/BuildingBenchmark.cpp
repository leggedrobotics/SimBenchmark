//
// Created by kangd on 09.05.18.
//

#include <MjcWorld_RG.hpp>

#include "BuildingBenchmark.hpp"
#include "MjcBenchmark.hpp"

mujoco_sim::MjcWorld_RG *sim;
po::options_description desc;

void setupSimulation() {
  if (benchmark::building::options.gui)
    sim = new mujoco_sim::MjcWorld_RG(800, 600, 0.05,
                                      benchmark::building::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption);
  else
    sim = new mujoco_sim::MjcWorld_RG(benchmark::building::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption);

  // time step
  sim->setTimeStep(benchmark::building::options.dt);

  /// no erp for mujoco
  if(benchmark::building::options.erpYN)
    RAIFATAL("erp is not supported for mujoco")

  // set up logger and timer
  if(benchmark::building::options.log)
    benchmark::building::loggerSetup(
        benchmark::building::getLogDirpath(benchmark::building::options.erpYN,
                                           benchmark::mujoco::options.simName,
                                           benchmark::mujoco::options.solverName,
                                           benchmark::building::options.dt), "var"
    );
}

void setupWorld() {

  if(benchmark::building::options.gui) {
    sim->setLightPosition((float)benchmark::building::params.lightPosition[0],
                          (float)benchmark::building::params.lightPosition[1],
                          (float)benchmark::building::params.lightPosition[2]);
    sim->cameraFollowObject(sim->getSingleBodyHandle(0), {0, 5, 2});
  }
}

void simulationLoop() {
  if(benchmark::building::options.gui) {
    // gui
    double numContact = 0;
    int i = 0;

    if(benchmark::building::options.saveVideo)
      sim->startRecordingVideo("/tmp", "mujoco-building");

    while(sim->visualizerLoop(benchmark::building::options.dt)) {
      sim->integrate();

      /// note that mujoco set object position after one step
      if(sim->getSingleBodyHandle(sim->getNumObject()-1)->getPosition()[2] <
          benchmark::building::params.heightLen * (benchmark::building::params.numFloor - 1) * 2) {
        // break if the building collapses
        RAIINFO("building collapsed!")
        break;
      }

      // calculate average contacts
      numContact = double(i) / double(i+1) * numContact + sim->getWorldNumContacts() / double(i+1);
      i++;
    }

    if(benchmark::building::options.saveVideo)
      sim->stopRecordingVideo();

    std::cout << "average contact " << numContact << "\n";
  }
  else {
    // no gui
    double numContact = 0;

    StopWatch watch;
    watch.start();

    int i = 0;
    for(i = 0; i < (int) (benchmark::building::options.T / benchmark::building::options.dt); i++) {
      sim->integrate();

      /// note that mujoco set object position after one step
      if(sim->getSingleBodyHandle(sim->getNumObject()-1)->getPosition()[2] <
          benchmark::building::params.heightLen * (benchmark::building::params.numFloor - 1) * 2) {
        // break if the building collapses
        RAIINFO("building collapsed!")
        break;
      }

      // calculate average contacts
      numContact = double(i) / double(i+1) * numContact + sim->getWorldNumContacts() / double(i+1);
    }

    // print to screen
    double time = watch.measure();
    std::cout << "time taken for " << i << " steps "<< time <<"s \n";
    std::cout << "average contact " << numContact << "\n";
  }
}

int main(int argc, const char* argv[]) {

  benchmark::building::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::building::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::building::getParamsFromYAML(benchmark::building::getYamlpath().c_str(),
                                         benchmark::MUJOCO);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::building::options.gui << std::endl
                << "ERP      : " << benchmark::building::options.erpYN << std::endl
                << "Timestep : " << benchmark::building::options.dt << std::endl
                << "Solver   : " << benchmark::mujoco::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  // time log
//  if(benchmark::building::options.log)
//    ru::timer->dumpToStdOuput();

  delete sim;
  return 0;
}

//
// Created by kangd on 15.02.18.
//

#include <MjcSim.hpp>

#include "BouncingBenchmark.hpp"
#include "MjcBenchmark.hpp"

mujoco_sim::MjcSim *sim;
po::options_description desc;

void setupSimulation() {
  if (benchmark::bouncing::options.gui)
    sim = new mujoco_sim::MjcSim(800, 600, 0.5,
                                      benchmark::bouncing::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);
  else
    sim = new mujoco_sim::MjcSim(benchmark::bouncing::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);

  // timestep
  sim->setTimeStep(benchmark::bouncing::options.dt);

  /// no erp for dart
  if(benchmark::bouncing::options.erpYN)
  RAIFATAL("erp is not supported for dart")

  // set up logger and timer
  if(benchmark::bouncing::options.log)
    benchmark::bouncing::loggerSetup(
        benchmark::bouncing::getLogDirpath(benchmark::bouncing::options.erpYN,
                                           benchmark::bouncing::options.e,
                                           benchmark::mujoco::options.simName,
                                           benchmark::mujoco::options.solverName,
                                           benchmark::bouncing::options.dt), "var"
    );
}

void resetWorld() {
  // materials
  // add objects
  int cnt = 0;
  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      auto ball = sim->getSingleBodyHandle(cnt + 1);
      ball->setFrictionCoefficient(benchmark::bouncing::params.mu_ball);

      if(benchmark::bouncing::options.gui)
        ball.visual()[0]->setColor(
            {benchmark::mujoco::color[0],
             benchmark::mujoco::color[1],
             benchmark::mujoco::color[2]});
      cnt++;
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::bouncing::params.g});

  if(benchmark::bouncing::options.gui) {
    sim->setLightPosition((float)benchmark::bouncing::params.lightPosition[0],
                          (float)benchmark::bouncing::params.lightPosition[1],
                          (float)benchmark::bouncing::params.lightPosition[2]);
    sim->cameraFollowObject(sim->getSingleBodyHandle(0), {10, 0, 10});
  }
}

void simulationLoop() {
  if(benchmark::bouncing::options.gui) {
    // gui
    if(benchmark::bouncing::options.saveVideo)
      sim->startRecordingVideo("/tmp", "mujoco-bouncing");

    for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt)
        && sim->visualizerLoop(benchmark::bouncing::options.dt); i++) {
      sim->integrate();

      // energy log
      if(benchmark::bouncing::options.log) {
        double energy = 0;

        /// j=0 is ground
        for(int j = 1; j < sim->getNumObject(); j++) {
          energy += sim->getSingleBodyHandle(j)->getEnergy({0, 0, benchmark::bouncing::params.g});
        }
        rai::Utils::logger->appendData("energy", energy);
      }
    }

    if(benchmark::bouncing::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    if(benchmark::bouncing::options.log)
      ru::timer->startTimer("bouncing");

    for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt); i++) {
      sim->integrate();

      if(benchmark::bouncing::options.log) {
        double energy = 0;

        /// j=0 is ground
        for(int j = 1; j < sim->getNumObject(); j++) {
          energy += sim->getSingleBodyHandle(j)->getEnergy({0, 0, benchmark::bouncing::params.g});
        }
        rai::Utils::logger->appendData("energy", energy);
      }
    }

    if(benchmark::bouncing::options.log)
      ru::timer->stopTimer("bouncing");
  }
}

int main(int argc, const char* argv[]) {

  benchmark::bouncing::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::bouncing::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlPath().c_str(),
                                         benchmark::DART);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::bouncing::options.gui << std::endl
                << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
                << "Res Coef : " << benchmark::bouncing::options.e << std::endl
                << "Timestep : " << benchmark::bouncing::options.dt << std::endl
                << "Solver   : " << benchmark::mujoco::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  resetWorld();
  simulationLoop();

  // time log
//  if(benchmark::bouncing::options.log)
//    ru::timer->dumpToStdOuput();

  delete sim;
  return 0;
}
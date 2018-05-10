//
// Created by kangd on 15.02.18.
//

#include "MjcWorld_RG.hpp"

#include "ThousandBenchmark.hpp"
#include "MjcBenchmark.hpp"

mujoco_sim::MjcWorld_RG *sim;
po::options_description desc;

void setupSimulation() {
  if (benchmark::thousand::options.gui)
    sim = new mujoco_sim::MjcWorld_RG(800, 600, 0.5,
                                      benchmark::thousand::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption);
  else
    sim = new mujoco_sim::MjcWorld_RG(benchmark::thousand::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption);

  // timestep and max contact
  sim->setTimeStep(benchmark::thousand::options.dt);

  /// no erp for dart
  if(benchmark::thousand::options.erpYN)
  RAIFATAL("erp is not supported for dart")

  // set up logger and timer
  if(benchmark::thousand::options.log)
    benchmark::thousand::loggerSetup(
        benchmark::thousand::getLogDirpath(benchmark::thousand::options.erpYN,
                                           benchmark::mujoco::options.simName,
                                           benchmark::mujoco::options.solverName,
                                           benchmark::thousand::options.dt), "var"
    );
}

void setupWorld() {

  const int n = benchmark::thousand::params.n;

  for(int i = 0; i < n; i++) {
    for(int j = 0; j < n; j++) {
      for(int k = 0; k < n; k++) {
        if(benchmark::thousand::options.gui) {
          if((i + j + k) % 3 == 0) {
            sim->getSingleBodyHandle(i * n * n + j * n + k + 1).visual()[0]->setColor(
                {benchmark::mujoco::color[0],
                 benchmark::mujoco::color[1],
                 benchmark::mujoco::color[2]});
          }
          else if((i + j + k) % 3 == 1) {
            sim->getSingleBodyHandle(i * n * n + j * n + k + 1).visual()[0]->setColor(
                {benchmark::mujoco::color[0],
                 benchmark::mujoco::color[1],
                 benchmark::mujoco::color[2]});
          }
          else if((i + j + k) % 3 == 2) {
            sim->getSingleBodyHandle(i * n * n + j * n + k + 1).visual()[0]->setColor(
                {benchmark::mujoco::color[0],
                 benchmark::mujoco::color[1],
                 benchmark::mujoco::color[2]});
          }
        }
      }
    }
  }

  if(benchmark::thousand::options.gui) {
    sim->setLightPosition((float)benchmark::thousand::params.lightPosition[0],
                          (float)benchmark::thousand::params.lightPosition[1],
                          (float)benchmark::thousand::params.lightPosition[2]);
    sim->cameraFollowObject(sim->getSingleBodyHandle(
        (sim->getNumObject() - 1) / 2), {0, 20, 10});
  }
}

double penetrationCheck() {
  double error = 0;
  int numObj = sim->getNumObject()-1;

  for (int i = 0; i < numObj; i++) {
    for (int j = i + 1; j < numObj; j++) {
      double dist = (sim->getSingleBodyHandle(i+1)->getPosition() - sim->getSingleBodyHandle(j+1)->getPosition()).norm();

      // error between spheres
      if (dist < benchmark::thousand::params.ballR * 2)
        error += (benchmark::thousand::params.ballR * 2 - dist) * (benchmark::thousand::params.ballR * 2 - dist);
    }

    // error sphere ~ ground
    if (sim->getSingleBodyHandle(i+1)->getPosition()[2] < benchmark::thousand::params.ballR) {
      error +=
          (benchmark::thousand::params.ballR - sim->getSingleBodyHandle(i+1)->getPosition()[2]) *
              (benchmark::thousand::params.ballR - sim->getSingleBodyHandle(i+1)->getPosition()[2]);
    }
  }

  return error;
}

void simulationLoop() {
  if(benchmark::thousand::options.timer)
    rai::Utils::timer->startTimer("thousand");

  if(benchmark::thousand::options.gui) {
    // gui
    if(benchmark::thousand::options.saveVideo)
      sim->startRecordingVideo("/tmp", "rai-building");

    for(int i = 0; i < (int) (benchmark::thousand::options.T / benchmark::thousand::options.dt)
        && sim->visualizerLoop(benchmark::thousand::options.dt); i++) {

      sim->integrate();

      if(benchmark::thousand::options.log) {
        double error = penetrationCheck();
        ru::logger->appendData("error", error);
      }
    }

    if(benchmark::thousand::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    for(int i = 0; i < (int) (benchmark::thousand::options.T / benchmark::thousand::options.dt); i++) {

      sim->integrate();

      if(benchmark::thousand::options.log) {
        double error = penetrationCheck();
        ru::logger->appendData("error", error);
      }
    }
  }

  if (benchmark::thousand::options.timer)
    rai::Utils::timer->stopTimer("thousand");
}

int main(int argc, const char* argv[]) {

  benchmark::thousand::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::thousand::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::thousand::getParamsFromYAML(benchmark::thousand::getYamlpath().c_str(),
                                         benchmark::MUJOCO);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::thousand::options.gui << std::endl
                << "ERP      : " << benchmark::thousand::options.erpYN << std::endl
                << "Timestep : " << benchmark::thousand::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  // time log
  if(benchmark::thousand::options.timer)
    ru::timer->dumpToStdOuput();

  delete sim;
  return 0;

}

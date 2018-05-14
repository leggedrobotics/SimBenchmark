//
// Created by kangd on 15.02.18.
//

#include "MjcWorld_RG.hpp"

#include "666Benchmark.hpp"
#include "MjcBenchmark.hpp"

mujoco_sim::MjcWorld_RG *sim;
po::options_description desc;

void setupSimulation() {
  if (benchmark::sixsixsix::options.gui)
    sim = new mujoco_sim::MjcWorld_RG(800, 600, 0.5,
                                      benchmark::sixsixsix::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption);
  else
    sim = new mujoco_sim::MjcWorld_RG(benchmark::sixsixsix::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption);

  // timestep and max contact
  sim->setTimeStep(benchmark::sixsixsix::options.dt);

  /// no erp for dart
  if(benchmark::sixsixsix::options.erpYN)
  RAIFATAL("erp is not supported for dart")
}

void setupWorld() {

  // gravity
  sim->setGravity({0, 0, benchmark::sixsixsix::params.g});

  const int n = benchmark::sixsixsix::params.n;

  for(int i = 0; i < n; i++) {
    for(int j = 0; j < n; j++) {
      for(int k = 0; k < n; k++) {
        if(benchmark::sixsixsix::options.gui) {
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

  if(benchmark::sixsixsix::options.gui) {
    sim->setLightPosition((float)benchmark::sixsixsix::params.lightPosition[0],
                          (float)benchmark::sixsixsix::params.lightPosition[1],
                          (float)benchmark::sixsixsix::params.lightPosition[2]);
    sim->cameraFollowObject(sim->getSingleBodyHandle(
        (sim->getNumObject() - 1) / 2), {0, 5, 2});
  }
}

double penetrationCheck() {
  double error = 0;
  int numObj = sim->getNumObject()-1;

  for (int i = 0; i < numObj; i++) {
    for (int j = i + 1; j < numObj; j++) {
      double dist = (sim->getSingleBodyHandle(i+1)->getPosition() - sim->getSingleBodyHandle(j+1)->getPosition()).norm();

      // error between spheres
      if (dist < benchmark::sixsixsix::params.ballR * 2)
        error += (benchmark::sixsixsix::params.ballR * 2 - dist) * (benchmark::sixsixsix::params.ballR * 2 - dist);
    }

    // error sphere ~ ground
    if (sim->getSingleBodyHandle(i+1)->getPosition()[2] < benchmark::sixsixsix::params.ballR) {
      error +=
          (benchmark::sixsixsix::params.ballR - sim->getSingleBodyHandle(i+1)->getPosition()[2]) *
              (benchmark::sixsixsix::params.ballR - sim->getSingleBodyHandle(i+1)->getPosition()[2]);
    }
  }

  return error;
}

void simulationLoop() {

  // init
  benchmark::sixsixsix::errorList.reserve(unsigned(benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt));

  // loop
  StopWatch watch;
  watch.start();
  if(benchmark::sixsixsix::options.gui) {
    // gui
    if(benchmark::sixsixsix::options.saveVideo)
      sim->startRecordingVideo("/tmp", "mujoco-666");

    for(int i = 0; i < (int) (benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt)
        && sim->visualizerLoop(benchmark::sixsixsix::options.dt); i++) {

      double error = penetrationCheck();
      benchmark::sixsixsix::errorList.push_back(error);
      sim->integrate();
    }

    if(benchmark::sixsixsix::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    for(int i = 0; i < (int) (benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt); i++) {

      double error = penetrationCheck();
      benchmark::sixsixsix::errorList.push_back(error);
      sim->integrate();
    }
  }
  double time = watch.measure();

  benchmark::sixsixsix::printError(0);
  if(benchmark::sixsixsix::options.log)
    benchmark::sixsixsix::printCSV(
        benchmark::sixsixsix::getLogFilepath(),
        benchmark::mujoco::options.simName,
        benchmark::mujoco::options.solverName,
        time,
        0
    );
}

int main(int argc, const char* argv[]) {

  benchmark::sixsixsix::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::sixsixsix::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::sixsixsix::getParamsFromYAML(benchmark::sixsixsix::getYamlpath().c_str(),
                                         benchmark::MUJOCO);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::sixsixsix::options.gui << std::endl
                << "ERP      : " << benchmark::sixsixsix::options.erpYN << std::endl
                << "Timestep : " << benchmark::sixsixsix::options.dt << std::endl
                << "Solver   : " << benchmark::mujoco::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  delete sim;
  return 0;

}

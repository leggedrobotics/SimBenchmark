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
}

void setupWorld() {
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

void resetWorld() {
  sim->resetSimulation();
  setupWorld();
}

double simulationLoop(bool timer = true, bool error = true) {
  // gui
  if(benchmark::bouncing::options.saveVideo)
    sim->startRecordingVideo("/tmp", "mjc-bouncing");

  // resever error vector
  benchmark::bouncing::data.setN(unsigned(benchmark::bouncing::params.T / benchmark::bouncing::options.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt); i++) {
    // gui
    if (benchmark::bouncing::options.gui && !sim->visualizerLoop(benchmark::bouncing::options.dt))
      break;

    // data save
    if (error) {
      double E = sim->getEnergy({0, 0, benchmark::bouncing::params.g});
      benchmark::bouncing::data.ballEnergy.push_back(E);
    }

    sim->integrate();
  }

  if(benchmark::bouncing::options.saveVideo)
    sim->stopRecordingVideo();

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
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

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::bouncing::data.computeError();

  // reset
  resetWorld();

  // trial2: get CPU time
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::bouncing::options.csv)
    benchmark::bouncing::printCSV(benchmark::bouncing::getCSVpath(),
                                  benchmark::mujoco::options.simName,
                                  benchmark::mujoco::options.solverName,
                                  benchmark::mujoco::options.detectorName,
                                  benchmark::mujoco::options.integratorName,
                                  time,
                                  error);

  RAIINFO(
      std::endl << "CPU time   : " << time << std::endl
                << "mean error : " << error << std::endl
                << "=======================" << std::endl
  )

  delete sim;
  return 0;
}
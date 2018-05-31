//
// Created by kangd on 15.02.18.
//

#include <DartSim.hpp>

#include "BouncingBenchmark.hpp"
#include "DartBenchmark.hpp"

dart_sim::DartSim *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::bouncing::options.gui)
    sim = new dart_sim::DartSim(800, 600, 0.5,
                                     benchmark::NO_BACKGROUND,
                                     benchmark::dart::options.solverOption);
  else
    sim = new dart_sim::DartSim(benchmark::dart::options.solverOption);

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
                                           benchmark::dart::options.simName,
                                           benchmark::dart::options.solverName,
                                           benchmark::bouncing::options.dt), "var"
    );
}

void setupWorld() {
  // materials
  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(benchmark::bouncing::params.mu_ground);
  checkerboard->setRestitutionCoefficient(1.0);

  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      auto ball = sim->addSphere(benchmark::bouncing::params.R, benchmark::bouncing::params.m);
      ball->setPosition(i * 2.0, j * 2.0, benchmark::bouncing::params.H);
      ball->setFrictionCoefficient(benchmark::bouncing::params.mu_ball);
      ball->setRestitutionCoefficient(benchmark::bouncing::options.e);

      if(benchmark::bouncing::options.gui)
        ball.visual()[0]->setColor(
            {benchmark::dart::color[0],
             benchmark::dart::color[1],
             benchmark::dart::color[2]});

      objList.push_back(ball);
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::bouncing::params.g});

  if(benchmark::bouncing::options.gui) {
    sim->setLightPosition((float)benchmark::bouncing::params.lightPosition[0],
                          (float)benchmark::bouncing::params.lightPosition[1],
                          (float)benchmark::bouncing::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {10, 0, 10});
  }
}

void simulationLoop() {
  if(benchmark::bouncing::options.gui) {
    // gui
    if(benchmark::bouncing::options.saveVideo)
      sim->startRecordingVideo("/tmp", "dart-bouncing");

    for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt)
        && sim->visualizerLoop(benchmark::bouncing::options.dt); i++) {
      sim->integrate();

      // energy log
      if(benchmark::bouncing::options.log) {
        double energy = 0;
        for(int j = 0; j < objList.size(); j++) {
          energy += objList[j]->getEnergy({0, 0, benchmark::bouncing::params.g});
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
        for(int j = 0; j < objList.size(); j++) {
          energy += objList[j]->getEnergy({0, 0, benchmark::bouncing::params.g});
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
  benchmark::dart::addDescToOption(desc);

  benchmark::bouncing::getOptionsFromArg(argc, argv, desc);
  benchmark::dart::getOptionsFromArg(argc, argv, desc);

  benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlpath().c_str(),
                                         benchmark::DART);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: DART" << std::endl
                << "GUI      : " << benchmark::bouncing::options.gui << std::endl
                << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
                << "Res Coef : " << benchmark::bouncing::options.e << std::endl
                << "Timestep : " << benchmark::bouncing::options.dt << std::endl
                << "Solver   : " << benchmark::dart::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  // time log
//  if(benchmark::bouncing::options.log)
//    ru::timer->dumpToStdOuput();

  delete sim;
  return 0;
}
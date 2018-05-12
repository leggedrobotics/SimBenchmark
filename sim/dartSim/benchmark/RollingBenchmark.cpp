//
// Created by kangd on 05.05.18.
//

#include "DartWorld_RG.hpp"

#include "RollingBenchmark.hpp"
#include "DartBenchmark.hpp"

dart_sim::DartWorld_RG *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::rolling::options.gui)
    sim = new dart_sim::DartWorld_RG(800, 600, 0.5,
                                     benchmark::NO_BACKGROUND,
                                     benchmark::dart::options.solverOption,
                                     benchmark::dart::options.detectorOption);
  else
    sim = new dart_sim::DartWorld_RG(benchmark::dart::options.solverOption,
                                     benchmark::dart::options.detectorOption);

  // timestep
  sim->setTimeStep(benchmark::rolling::options.dt);

  /// no erp for dart
  if(benchmark::rolling::options.erpYN)
    RAIFATAL("erp is not supported for dart")

  // set up logger and timer
  if(benchmark::rolling::options.log)
    benchmark::rolling::loggerSetup(
        benchmark::rolling::getLogDirpath(benchmark::rolling::options.erpYN,
                                          benchmark::rolling::options.forceDirection,
                                          benchmark::dart::options.simName,
                                          benchmark::dart::options.solverName,
                                          benchmark::rolling::options.dt), "var"
    );
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(benchmark::rolling::params.dartGroundMu);

  auto box = sim->addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5 - benchmark::rolling::params.initPenetration);
  box->setFrictionCoefficient(benchmark::rolling::params.dartBoxMu);
  objList.push_back(box);

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0,
                        j * 2.0 - 4.0,
                        1.5 - 3 * benchmark::rolling::params.initPenetration);
      ball->setFrictionCoefficient(benchmark::rolling::params.dartBallMu);
      objList.push_back(ball);
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::rolling::params.g});

  if(benchmark::rolling::options.gui) {
    sim->setLightPosition((float)benchmark::rolling::params.lightPosition[0],
                          (float)benchmark::rolling::params.lightPosition[1],
                          (float)benchmark::rolling::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {30, 0, 15});
  }
}

void simulationLoop() {

  // force
  Eigen::Vector3d force;
  if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_Y)
    force = {0,
             benchmark::rolling::params.F,
             0};
  else if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_XY)
    force = {benchmark::rolling::params.F * 0.707106781186547,
             benchmark::rolling::params.F * 0.707106781186547,
             0};

  if(benchmark::rolling::options.gui) {
    // gui
    if(benchmark::rolling::options.saveVideo)
      sim->startRecordingVideo("/tmp", "dart-rolling");

    for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt) &&
        sim->visualizerLoop(benchmark::rolling::options.dt); i++) {

      // set force to box
      objList[0]->setExternalForce(force);

      // log
      if(benchmark::rolling::options.log) {
        ru::logger->appendData("velbox", objList[0]->getLinearVelocity().data());
        ru::logger->appendData("velball", objList[1]->getLinearVelocity().data());
        ru::logger->appendData("posbox", objList[0]->getPosition().data());
        ru::logger->appendData("posball", objList[1]->getPosition().data());
      }

      sim->integrate();
    }

    if(benchmark::rolling::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    if(benchmark::rolling::options.log)
      ru::timer->startTimer("rolling");

    for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt); i++) {

      // set force to box
      objList[0]->setExternalForce(force);

      // log
      if(benchmark::rolling::options.log) {
        ru::logger->appendData("velbox", objList[0]->getLinearVelocity().data());
        ru::logger->appendData("velball", objList[1]->getLinearVelocity().data());
        ru::logger->appendData("posbox", objList[0]->getPosition().data());
        ru::logger->appendData("posball", objList[1]->getPosition().data());
      }

      sim->integrate();
    }

    if(benchmark::rolling::options.log)
      ru::timer->stopTimer("rolling");
  }
}

int main(int argc, const char* argv[]) {

  benchmark::rolling::addDescToOption(desc);
  benchmark::dart::addDescToOption(desc);

  benchmark::rolling::getOptionsFromArg(argc, argv, desc);
  benchmark::dart::getOptionsFromArg(argc, argv, desc);

  benchmark::rolling::getParamsFromYAML(benchmark::rolling::getYamlpath().c_str(),
                                        benchmark::DART);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: DART" << std::endl
                << "GUI      : " << benchmark::rolling::options.gui << std::endl
                << "ERP      : " << benchmark::rolling::options.erpYN << std::endl
                << "Force    : " << benchmark::rolling::options.forceDirection << std::endl
                << "Timestep : " << benchmark::rolling::options.dt << std::endl
                << "Solver   : " << benchmark::dart::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  // time log
  if(benchmark::rolling::options.log)
    ru::timer->dumpToStdOuput();

  delete sim;
  return 0;
}
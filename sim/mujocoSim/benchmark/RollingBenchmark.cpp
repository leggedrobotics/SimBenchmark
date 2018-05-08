//
// Created by kangd on 21.02.18.
//

#include <MjcWorld_RG.hpp>

#include "MjcBenchmark.hpp"
#include "RollingBenchmark.hpp"

mujoco_sim::MjcWorld_RG *sim;
po::options_description desc;

void setupSimulation() {
  if (benchmark::rolling::options.gui)
    sim = new mujoco_sim::MjcWorld_RG(800, 600, 0.5,
                                      benchmark::rolling::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption);
  else
    sim = new mujoco_sim::MjcWorld_RG(benchmark::rolling::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption);

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
                                          benchmark::mujoco::options.simName,
                                          benchmark::mujoco::options.solverName,
                                          benchmark::rolling::options.dt), "var"
    );
}

void setupWorld() {

  // gravity
  sim->setGravity({0, 0, benchmark::rolling::params.g});

  /// Note. for mujoco (frictional coefficient A-B) = max(coeff of A, coeff of B)
  sim->getSingleBodyHandle(0)->setFrictionCoefficient(benchmark::rolling::params.mjcGroundMu);
  sim->getSingleBodyHandle(1)->setFrictionCoefficient(benchmark::rolling::params.mjcBoxMu);
  for(int i = 2; i < sim->getNumObject(); i++) {
    sim->getSingleBodyHandle(1)->setFrictionCoefficient(benchmark::rolling::params.mjcBallMu);
  }

  if(benchmark::rolling::options.gui) {
    sim->setLightPosition((float)benchmark::rolling::params.lightPosition[0],
                          (float)benchmark::rolling::params.lightPosition[1],
                          (float)benchmark::rolling::params.lightPosition[2]);
    sim->cameraFollowObject(sim->getSingleBodyHandle(0), {30, 0, 15});
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
      sim->startRecordingVideo("/tmp", "mujoco-rolling");

    for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt) &&
        sim->visualizerLoop(benchmark::rolling::options.dt); i++) {

      // set force to box
      sim->getSingleBodyHandle(1)->setExternalForce(force);

      // log
      if(benchmark::rolling::options.log) {
        ru::logger->appendData("velbox", sim->getSingleBodyHandle(1)->getLinearVelocity().data());
        ru::logger->appendData("velball", sim->getSingleBodyHandle(2)->getLinearVelocity().data());
        ru::logger->appendData("posbox", sim->getSingleBodyHandle(1)->getPosition().data());
        ru::logger->appendData("posball", sim->getSingleBodyHandle(2)->getPosition().data());
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
      sim->getSingleBodyHandle(1)->setExternalForce(force);

      // log
      if(benchmark::rolling::options.log) {
        ru::logger->appendData("velbox", sim->getSingleBodyHandle(1)->getLinearVelocity().data());
        ru::logger->appendData("velball", sim->getSingleBodyHandle(2)->getLinearVelocity().data());
        ru::logger->appendData("posbox", sim->getSingleBodyHandle(1)->getPosition().data());
        ru::logger->appendData("posball", sim->getSingleBodyHandle(2)->getPosition().data());
      }

      sim->integrate();
    }

    if(benchmark::rolling::options.log)
      ru::timer->stopTimer("rolling");
  }
}

int main(int argc, const char* argv[]) {

  benchmark::rolling::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::rolling::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::rolling::getParamsFromYAML(benchmark::rolling::getYamlpath().c_str(),
                                        benchmark::MUJOCO);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::rolling::options.gui << std::endl
                << "ERP      : " << benchmark::rolling::options.erpYN << std::endl
                << "Force    : " << benchmark::rolling::options.forceDirection << std::endl
                << "Timestep : " << benchmark::rolling::options.dt << std::endl
                << "Solver   : " << benchmark::mujoco::options.solverName << std::endl
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
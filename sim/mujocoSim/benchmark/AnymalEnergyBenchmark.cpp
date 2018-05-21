//
// Created by kangd on 14.05.18.
//

#include <MjcWorld_RG.hpp>

#include "AnymalEnergyBenchmark.hpp"
#include "MjcBenchmark.hpp"

mujoco_sim::MjcWorld_RG *sim;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::freedrop::options.gui)
    sim = new mujoco_sim::MjcWorld_RG(800, 600, 0.5,
                                      benchmark::anymal::freedrop::getMujocoURDFpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);
  else
    sim = new mujoco_sim::MjcWorld_RG(benchmark::anymal::freedrop::getMujocoURDFpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);

  // set time step
  sim->setTimeStep(benchmark::anymal::freedrop::options.dt);
}

void setupWorld() {

  sim->setGeneralizedCoordinate({0, 0, benchmark::anymal::freedrop::params.H,
                                 1.0, 0.0, 0.0, 0.0,
                                 0.03, 0.4, -0.8,
                                 0.03, -0.4, +0.8,
                                 -0.03, 0.4, -0.8,
                                 -0.03, -0.4, 0.8});
  sim->setGeneralizedForce(Eigen::VectorXd::Zero(sim->getDOF()));

  // gravity
  sim->setGravity({0, 0, benchmark::anymal::freedrop::params.g});

  // mass
  benchmark::anymal::freedrop::params.M = sim->getTotalMass();
  benchmark::anymal::freedrop::params.F =
      benchmark::anymal::freedrop::params.M * (-benchmark::anymal::freedrop::params.g) * 2;

  // collision mask (no internal collision)
  for(int i = 0 ; i < sim->getNumObject(); i++) {
    static_cast<mujoco_sim::object::MjcSingleBodyObject *>(sim->getSingleBodyHandle(i).s_)
        ->setCollisionGroupAndMask(1, 0);
  }

  if(benchmark::anymal::freedrop::options.gui)
    sim->cameraFollowObject(
        sim->getSingleBodyHandle(sim->getNumObject()-1), {25.0, 0.0, 7.0});
}

double computeEnergy() {
  return sim->getEnergy({0, 0, benchmark::anymal::freedrop::params.g});
}

double computeEnergyError(double E0) {
  // compute linear momentum
  return pow(computeEnergy() - E0, 2);
}

double simulationLoop() {

  // error list
  benchmark::anymal::freedrop::data.errorList.reserve(
      unsigned(benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt));
  benchmark::anymal::freedrop::data.EList.reserve(
      unsigned(benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt));

  StopWatch watch;
  watch.start();
  if(benchmark::anymal::freedrop::options.gui) {
    // gui
    if(benchmark::anymal::freedrop::options.saveVideo)
      sim->startRecordingVideo("/tmp", "mjc-anymal-freedrop");

    // step1: applying force
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor); t++) {

      sim->integrate1();
      sim->setGeneralizedForce({0, 0, benchmark::anymal::freedrop::params.F,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0});
      sim->integrate2();
    }

    // step2: freedrop
    double E0 = 0;
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor); t++) {

      sim->forwardKinematics();
      if(t == 0)
        E0 = computeEnergy();

      sim->setGeneralizedForce({0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0});
      benchmark::anymal::freedrop::data.errorList.push_back(computeEnergyError(E0));
      benchmark::anymal::freedrop::data.EList.push_back(computeEnergy());
      sim->integrate();
    }

    if(benchmark::anymal::freedrop::options.saveVideo)
      sim->stopRecordingVideo();

  } else {
    // step1: applying force
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt); t++) {

      sim->integrate1();
      sim->setGeneralizedForce({0, 0, benchmark::anymal::freedrop::params.F,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0});
      sim->integrate2();
    }

    // step2: freedrop
    double E0 = 0;
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt); t++) {

      sim->forwardKinematics();
      if(t == 0)
        E0 = computeEnergy();
      sim->setGeneralizedForce({0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0});
      benchmark::anymal::freedrop::data.errorList.push_back(computeEnergyError(E0));
      benchmark::anymal::freedrop::data.EList.push_back(computeEnergy());
      sim->integrate();
    }
  }

  double time = watch.measure();
  if(benchmark::anymal::freedrop::options.csv)
    benchmark::anymal::freedrop::printCSV(benchmark::anymal::freedrop::getCSVpath(),
                                          benchmark::mujoco::options.simName,
                                          benchmark::mujoco::options.solverName,
                                          benchmark::mujoco::options.detectorName,
                                          benchmark::mujoco::options.integratorName,
                                          time);
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::freedrop::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::freedrop::getParamsFromYAML(benchmark::anymal::freedrop::getYamlpath().c_str(),
                                                 benchmark::MUJOCO);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::anymal::freedrop::options.gui << std::endl
                << "Solver   : " << benchmark::mujoco::options.solverOption << std::endl
                << "Timestep : " << benchmark::anymal::freedrop::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();

  RAIINFO(
      std::endl << "Timer    : " << simulationLoop() << std::endl
                << "Mean Error: " << benchmark::anymal::freedrop::computeMeanError() << std::endl
                << "Contacts  : " << sim->getWorldNumContacts() << std::endl
                << "======================="
  )

  // show plot
  if(benchmark::anymal::freedrop::options.plot) {
    benchmark::anymal::freedrop::showPlot();
  }

  delete sim;
  return 0;
}
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
                                      benchmark::mujoco::options.solverOption);
  else
    sim = new mujoco_sim::MjcWorld_RG(benchmark::anymal::freedrop::getMujocoURDFpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption);

  // set time step
  sim->setTimeStep(benchmark::anymal::freedrop::options.dt);
}

void setupWorld() {

  sim->setGeneralizedCoordinate({0,
                                 0,
                                 benchmark::anymal::freedrop::params.H,
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

  if(benchmark::anymal::freedrop::options.gui)
    sim->cameraFollowObject(
        sim->getSingleBodyHandle(sim->getNumObject()-1), {10.0, 0.0, 30.0});
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
  benchmark::anymal::freedrop::errorList.reserve(
      unsigned(benchmark::anymal::freedrop::params.T / benchmark::anymal::freedrop::options.dt));

  // E0
  sim->forwardKinematics();
  double E0 = computeEnergy();

  StopWatch watch;
  watch.start();
  if(benchmark::anymal::freedrop::options.gui) {
    // gui
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T / benchmark::anymal::freedrop::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor); t++) {

      sim->integrate1();
      benchmark::anymal::freedrop::errorList.push_back(computeEnergyError(E0));
      sim->integrate2();
    }
  } else {
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T / benchmark::anymal::freedrop::options.dt); t++) {
      sim->integrate1();
      benchmark::anymal::freedrop::errorList.push_back(computeEnergyError(E0));
      sim->integrate2();
    }
  }

  double time = watch.measure();
  if(benchmark::anymal::freedrop::options.csv)
    benchmark::anymal::freedrop::printCSV(
        benchmark::anymal::freedrop::getCSVpath(),
        benchmark::mujoco::options.simName,
        benchmark::mujoco::options.solverName,
        time);
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::freedrop::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

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
                << "======================="
  )

  // show plot
  if(benchmark::anymal::freedrop::options.plot) {
    benchmark::anymal::freedrop::showPlot();
  }

  delete sim;
  return 0;
}
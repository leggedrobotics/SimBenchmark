//
// Created by kangd on 14.05.18.
//

#include <MjcSim.hpp>

#include "AnymalMomentumBenchmark.hpp"
#include "MjcBenchmark.hpp"

mujoco_sim::MjcSim *sim;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::zerogravity::options.gui)
    sim = new mujoco_sim::MjcSim(800, 600, 0.5,
                                      benchmark::anymal::zerogravity::getMujocoURDFpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);
  else
    sim = new mujoco_sim::MjcSim(benchmark::anymal::zerogravity::getMujocoURDFpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);

  // set time step
  sim->setTimeStep(benchmark::anymal::zerogravity::options.dt);
}

void setupWorld() {

  Eigen::VectorXd genCoord(26);
  genCoord << 0, 0, benchmark::anymal::zerogravity::params.H,
      1.0, 0.0, 0.0, 0.0,
      0.03, 0.4, -0.8,
      -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8,
      -0.03, -0.4, 0.8,
      0, benchmark::anymal::zerogravity::params.x0, benchmark::anymal::zerogravity::params.H,
      1.0, 0.0, 0.0, 0.0;

  Eigen::VectorXd genVelocity(24);
  genVelocity << 0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, benchmark::anymal::zerogravity::params.v0, 0,
      0, 0, 0;

  sim->setGeneralizedCoordinate(genCoord);
  sim->setGeneralizedVelocity(genVelocity);
  sim->setGeneralizedForce(Eigen::VectorXd::Zero(sim->getDOF()));

  // gravity
  sim->setGravity({0, 0, 0});

  // mass
  benchmark::anymal::zerogravity::params.M = sim->getTotalMass() - benchmark::anymal::zerogravity::params.m;
  if(benchmark::anymal::zerogravity::options.gui)
    sim->cameraFollowObject(
        sim->getSingleBodyHandle(sim->getNumObject()-1), {10.0, 0.0, 1.0});  // focus on ground
}

double computeLinearMomentumError() {
  // compute linear momentum
  Eigen::Vector3d linearMomentum = sim->getLinearMomentumInCartesianSpace();
  Eigen::Vector3d analyticSol(0, benchmark::anymal::zerogravity::params.m * benchmark::anymal::zerogravity::params.v0, 0);

  return pow((linearMomentum - analyticSol).norm(), 2);
}


double simulationLoop() {

  // error list
  benchmark::anymal::zerogravity::errorList.reserve(
      unsigned(benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt));

  StopWatch watch;
  watch.start();
  if(benchmark::anymal::zerogravity::options.gui) {
    // gui
    for (int t = 0; t < (int) (benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::zerogravity::options.dt, 1.0); t++) {

      sim->forwardKinematics();
      benchmark::anymal::zerogravity::errorList.push_back(computeLinearMomentumError());
      sim->integrate();
    }
  } else {
    for (int t = 0; t < (int) (benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt); t++) {

      sim->forwardKinematics();
      benchmark::anymal::zerogravity::errorList.push_back(computeLinearMomentumError());
      sim->integrate2();
    }
  }

  double time = watch.measure();
  if(benchmark::anymal::zerogravity::options.csv)
    benchmark::anymal::zerogravity::printCSV(benchmark::anymal::zerogravity::getCSVpath(),
                                             benchmark::mujoco::options.simName,
                                             benchmark::mujoco::options.solverName,
                                             benchmark::mujoco::options.detectorName,
                                             benchmark::mujoco::options.integratorName,
                                             time);
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::zerogravity::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::anymal::zerogravity::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::zerogravity::getParamsFromYAML(benchmark::anymal::zerogravity::getYamlpath().c_str(),
                                                    benchmark::MUJOCO);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::anymal::zerogravity::options.gui << std::endl
                << "Solver   : " << benchmark::mujoco::options.solverOption << std::endl
                << "Timestep : " << benchmark::anymal::zerogravity::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();

  RAIINFO(
      std::endl << "Timer    : " << simulationLoop() << std::endl
                << "Mean Error: " << benchmark::anymal::zerogravity::computeMeanError() << std::endl
                << "======================="
  )

  // show plot
  if(benchmark::anymal::zerogravity::options.plot) {
    benchmark::anymal::zerogravity::showPlot();
  }

  delete sim;
  return 0;
}
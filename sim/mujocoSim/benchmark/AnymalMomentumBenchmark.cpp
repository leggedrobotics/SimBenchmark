//
// Created by kangd on 14.05.18.
//

#include <MjcWorld_RG.hpp>

#include "AnymalMomentumBenchmark.hpp"
#include "MjcBenchmark.hpp"

mujoco_sim::MjcWorld_RG *sim;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::zerogravity::options.gui)
    sim = new mujoco_sim::MjcWorld_RG(800, 600, 0.5,
                                      benchmark::anymal::zerogravity::getMujocoURDFpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption);
  else
    sim = new mujoco_sim::MjcWorld_RG(benchmark::anymal::zerogravity::getMujocoURDFpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption);

  // set time step
  sim->setTimeStep(benchmark::anymal::zerogravity::options.dt);
}

void setupWorld() {

  Eigen::VectorXd genCoord(38);
  genCoord << 0, benchmark::anymal::zerogravity::params.x0, benchmark::anymal::zerogravity::params.H,
      1.0, 0.0, 0.0, 0.0,
      0.03, 0.4, -0.8,
      -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8,
      -0.03, -0.4, 0.8,
      0, 0, benchmark::anymal::zerogravity::params.H,
      1.0, 0.0, 0.0, 0.0,
      0.03, 0.4, -0.8,
      -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8,
      -0.03, -0.4, 0.8;

  Eigen::VectorXd genVelocity(36);
  genVelocity << 0, benchmark::anymal::zerogravity::params.v0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0;

  sim->setGeneralizedCoordinate(genCoord);
  sim->setGeneralizedVelocity(genVelocity);
  sim->setGeneralizedForce(Eigen::VectorXd::Zero(sim->getDOF()));

  // gravity
  sim->setGravity({0, 0, 0});

  // mass
  benchmark::anymal::zerogravity::params.M = sim->getTotalMass() * 0.5; /// since two anymals
  if(benchmark::anymal::zerogravity::options.gui)
    sim->cameraFollowObject(
        sim->getSingleBodyHandle(26), {10.0, 0.0, 1.0});  // focus on ground
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

      sim->integrate();

      benchmark::anymal::zerogravity::errorList.push_back(
          pow((sim->getLinearMomentumInCartesianSpace()
              - Eigen::Vector3d(0,
                                benchmark::anymal::zerogravity::params.M * benchmark::anymal::zerogravity::params.v0,
                                0)).norm(), 2)
      );
    }
  } else {
    for (int t = 0; t < (int) (benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt); t++) {
      sim->integrate();

      benchmark::anymal::zerogravity::errorList.push_back(
          pow((sim->getLinearMomentumInCartesianSpace()
              - Eigen::Vector3d(0,
                                benchmark::anymal::zerogravity::params.M * benchmark::anymal::zerogravity::params.v0,
                                0)).norm(), 2)
      );
    }
  }

  return watch.measure();
}

double computeMeanError() {
  return std::accumulate(benchmark::anymal::zerogravity::errorList.begin(),
                         benchmark::anymal::zerogravity::errorList.end(), 0.0)
      / benchmark::anymal::zerogravity::errorList.size();
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::zerogravity::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::anymal::zerogravity::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

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
                << "Mean Error: " << computeMeanError() << std::endl
                << "======================="
  )

  delete sim;
  return 0;
}
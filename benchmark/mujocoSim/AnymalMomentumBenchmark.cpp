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

void resetWorld() {
  sim->resetSimulation();
}

double simulationLoop(bool timer = true, bool error = true) {
  if(benchmark::anymal::zerogravity::options.gui && benchmark::anymal::zerogravity::options.saveVideo)
    sim->startRecordingVideo("/tmp", "bullet-anymal-momentum");

  // resever error vector
  if(error)
    benchmark::anymal::zerogravity::data.setN(
        unsigned(benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt)
    );

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();
  for(int i = 0; i < (int) (benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt); i++) {
    // gui
    if(benchmark::anymal::zerogravity::options.gui && !sim->visualizerLoop(benchmark::anymal::zerogravity::options.dt))
      break;

    // integrate step1
    sim->forwardKinematics();

    // data save
    if(error) {
      Eigen::Vector3d ballM = sim->getSingleBodyHandle(0)->getLinearMomentum();
      Eigen::Vector3d totalM = sim->getLinearMomentumInCartesianSpace();

      benchmark::anymal::zerogravity::data.ballMomentum.push_back(
          ballM
      );
      benchmark::anymal::zerogravity::data.anymalMomentum.push_back(
          (totalM - ballM)
      );
    }

    // step 2
    sim->integrate();
  }

  double time = 0;
  if(timer)
    time = watch.measure();
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

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::anymal::zerogravity::data.computeError();

  // reset
  resetWorld();

  // trial2: get CPU time
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::anymal::zerogravity::options.csv)
    benchmark::anymal::zerogravity::printCSV(benchmark::anymal::zerogravity::getCSVpath(),
                                             benchmark::mujoco::options.simName,
                                             benchmark::mujoco::options.solverName,
                                             benchmark::mujoco::options.detectorName,
                                             benchmark::mujoco::options.integratorName,
                                             time,
                                             error);

  RAIINFO(
      std::endl << "CPU Timer : " << time << std::endl
                << "Mean Error: " << error << std::endl
                << "======================="
  )

  delete sim;
  return 0;
}
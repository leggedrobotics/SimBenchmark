//
// Created by kangd on 14.05.18.
//

#include <BtMbSim.hpp>

#include "AnymalMomentumBenchmark.hpp"
#include "BtMbBenchmark.hpp"

bullet_mb_sim::BtMbSim *sim;
std::vector<benchmark::SingleBodyHandle> balls;
std::vector<bullet_mb_sim::ArticulatedSystemHandle> anymals;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::zerogravity::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  // set erp 0
  sim->setERP(0, 0, 0);
  sim->setTimeStep(benchmark::anymal::zerogravity::options.dt);
}

void resetWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

  // ball
  auto ball = sim->addSphere(0.2, benchmark::anymal::zerogravity::params.m);
  ball->setPosition(0,
                    benchmark::anymal::zerogravity::params.x0,
                    benchmark::anymal::zerogravity::params.H);
  ball->setVelocity(0, benchmark::anymal::zerogravity::params.v0, 0, 0, 0, 0);
  balls.push_back(ball);

  // anymal
  auto anymal =
      sim->addArticulatedSystem(benchmark::anymal::zerogravity::getURDFpath(), bullet_mb_sim::object::URDF);
  anymal->setGeneralizedCoordinate({0,
                                    0,
                                    benchmark::anymal::zerogravity::params.H,
                                    1.0, 0.0, 0.0, 0.0,
                                    0.03, 0.4, -0.8,
                                    -0.03, 0.4, -0.8,
                                    0.03, -0.4, 0.8,
                                    -0.03, -0.4, 0.8});

  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymals.push_back(anymal);

  // gravity
  sim->setGravity({0, 0, 0});

  // mass
  benchmark::anymal::zerogravity::params.M = anymal->getTotalMass();
  if(benchmark::anymal::zerogravity::options.gui)
    sim->cameraFollowObject(checkerboard, {10.0, 0.0, 1.0});
}

double computeLinearMomentumError() {
  // compute linear momentum
  Eigen::Vector3d linearMomentum;
  linearMomentum.setZero();

  for(int i = 0; i < anymals.size(); i++) {
    linearMomentum += anymals[i]->getLinearMomentumInCartesianSpace();
  }
  for(int i = 0; i < balls.size(); i++) {
    linearMomentum += balls[i]->getLinearMomentum();
  }

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
    if(benchmark::anymal::zerogravity::options.saveVideo)
      sim->startRecordingVideo("/tmp", "bullet-zero-gravity");

    for (int t = 0; t < (int) (benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::zerogravity::options.dt, 1.0); t++) {

      benchmark::anymal::zerogravity::errorList.push_back(computeLinearMomentumError());
      sim->integrate();
    }

    if(benchmark::anymal::zerogravity::options.saveVideo)
      sim->stopRecordingVideo();

  } else {
    for (int t = 0; t < (int) (benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt); t++) {
      benchmark::anymal::zerogravity::errorList.push_back(computeLinearMomentumError());
      sim->integrate();
    }
  }

  double time = watch.measure();
  if(benchmark::anymal::zerogravity::options.csv)
    benchmark::anymal::zerogravity::printCSV(benchmark::anymal::zerogravity::getCSVpath(),
                                             benchmark::bulletmultibody::options.simName,
                                             benchmark::bulletmultibody::options.solverName,
                                             benchmark::bulletmultibody::options.detectorName,
                                             benchmark::bulletmultibody::options.integratorName,
                                             time);
  return time;
}

double computeMeanError() {
  return std::accumulate(benchmark::anymal::zerogravity::errorList.begin(),
                         benchmark::anymal::zerogravity::errorList.end(), 0.0)
      / benchmark::anymal::zerogravity::errorList.size();
}

int main(int argc, const char* argv[]) {


  benchmark::anymal::zerogravity::addDescToOption(desc);
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::anymal::zerogravity::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::zerogravity::getParamsFromYAML(benchmark::anymal::zerogravity::getYamlpath().c_str(),
                                                    benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: BULLET" << std::endl
                << "GUI      : " << benchmark::anymal::zerogravity::options.gui << std::endl
                << "Solver   : " << benchmark::bulletmultibody::options.solverName << std::endl
                << "Timestep : " << benchmark::anymal::zerogravity::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  resetWorld();

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
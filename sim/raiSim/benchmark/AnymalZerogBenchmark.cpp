//
// Created by kangd on 14.05.18.
//

#include <raiSim/World_RG.hpp>

#include "AnymalZerogBenchmark.hpp"

rai_sim::World_RG *sim;
std::vector<rai_sim::ArticulatedSystemHandle> anymals;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::zerogravity::options.gui)
    sim = new rai_sim::World_RG(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  else
    sim = new rai_sim::World_RG();
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, 1, -1, rai_sim::GRID);

  auto anymal = sim->addArticulatedSystem(
      benchmark::anymal::zerogravity::getURDFpath()
  );
  anymal->setGeneralizedCoordinate({0,
                                    benchmark::anymal::zerogravity::params.x0,
                                    benchmark::anymal::zerogravity::params.H,
                                    1.0, 0.0, 0.0, 0.0,
                                    0.03, 0.4, -0.8,
                                    -0.03, 0.4, -0.8,
                                    0.03, -0.4, 0.8,
                                    -0.03, -0.4, 0.8});
  anymal->setGeneralizedVelocity({0, benchmark::anymal::zerogravity::params.v0, 0,
                                  0, 0, 0,
                                  0, 0, 0,
                                  0, 0, 0,
                                  0, 0, 0,
                                  0, 0, 0});
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymals.push_back(anymal);

  auto anymal2 = sim->addArticulatedSystem(
      benchmark::anymal::zerogravity::getURDFpath()
  );
  anymal2->setGeneralizedCoordinate({0,
                                     0,
                                     benchmark::anymal::zerogravity::params.H,
                                     1.0, 0.0, 0.0, 0.0,
                                     0.03, 0.4, -0.8,
                                     -0.03, 0.4, -0.8,
                                     0.03, -0.4, 0.8,
                                     -0.03, -0.4, 0.8});
  anymal2->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal2->getDOF()));
  anymal2->setGeneralizedForce(Eigen::VectorXd::Zero(anymal2->getDOF()));
  anymals.push_back(anymal2);

  // gravity
  sim->setGravity({0, 0, 0});

  // mass
  benchmark::anymal::zerogravity::params.m =
      std::accumulate( anymal->getMass().begin(), anymal->getMass().end(), 0.0);

  if(benchmark::anymal::zerogravity::options.gui)
    sim->cameraFollowObject(checkerboard, {10.0, 0.0, 1.0});
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

      sim->integrate(benchmark::anymal::zerogravity::options.dt);

      Eigen::Vector3d linearMomentum;
      linearMomentum.setZero();
      for(int i = 0; i < anymals.size(); i++) {
        linearMomentum += anymals[i]->getLinearMomentumInCartesianSpace();
      }
      benchmark::anymal::zerogravity::errorList.push_back(
          pow((linearMomentum
              - Eigen::Vector3d(0,
                                benchmark::anymal::zerogravity::params.m * benchmark::anymal::zerogravity::params.v0,
                                0)).norm(), 2)
      );
    }
  } else {
    for (int t = 0; t < (int) (benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt); t++) {
      sim->integrate(benchmark::anymal::zerogravity::options.dt);

      Eigen::Vector3d linearMomentum;
      linearMomentum.setZero();
      for(int i = 0; i < anymals.size(); i++) {
        linearMomentum += anymals[i]->getLinearMomentumInCartesianSpace();
      }
      benchmark::anymal::zerogravity::errorList.push_back(
          pow((linearMomentum
              - Eigen::Vector3d(0,
                                benchmark::anymal::zerogravity::params.m * benchmark::anymal::zerogravity::params.v0,
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
  benchmark::anymal::zerogravity::getOptionsFromArg(argc, argv, desc);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::anymal::zerogravity::options.gui << std::endl
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
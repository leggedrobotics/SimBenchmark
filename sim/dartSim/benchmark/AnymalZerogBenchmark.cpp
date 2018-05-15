//
// Created by kangd on 14.05.18.
//

#include <DartWorld_RG.hpp>

#include "AnymalZerogBenchmark.hpp"
#include "DartBenchmark.hpp"

dart_sim::DartWorld_RG *sim;
std::vector<dart_sim::ArticulatedSystemHandle> anymals;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::zerogravity::options.gui)
    sim = new dart_sim::DartWorld_RG(800, 600, 0.5,
                                     benchmark::NO_BACKGROUND,
                                     benchmark::dart::options.solverOption);
  else
    sim = new dart_sim::DartWorld_RG(benchmark::dart::options.solverOption);

  // set time step
  sim->setTimeStep(benchmark::anymal::zerogravity::options.dt);
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

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
  benchmark::anymal::zerogravity::params.m = anymal->getTotalMass();
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

      sim->integrate();

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
      sim->integrate();

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
  benchmark::dart::addDescToOption(desc);

  benchmark::anymal::zerogravity::getOptionsFromArg(argc, argv, desc);
  benchmark::dart::getOptionsFromArg(argc, argv, desc);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: DART" << std::endl
                << "GUI      : " << benchmark::anymal::zerogravity::options.gui << std::endl
                << "Solver   : " << benchmark::dart::options.solverOption << std::endl
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
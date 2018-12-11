//
// Created by kangd on 14.05.18.
//

#include <DartSim.hpp>

#include "AnymalMomentumBenchmark.hpp"
#include "DartBenchmark.hpp"

dart_sim::DartSim *sim;
std::vector<benchmark::SingleBodyHandle> balls;
std::vector<dart_sim::ArticulatedSystemHandle> anymals;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::zerogravity::options.gui)
    sim = new dart_sim::DartSim(800, 600, 0.5,
                                     benchmark::NO_BACKGROUND,
                                     benchmark::dart::options.solverOption);
  else
    sim = new dart_sim::DartSim(benchmark::dart::options.solverOption);

  // set time step
  sim->setTimeStep(benchmark::anymal::zerogravity::options.dt);
}

void setupWorld() {
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
  auto anymal = sim->addArticulatedSystem(
      benchmark::anymal::zerogravity::getURDFpath()
  );
  anymal->setGeneralizedCoordinate({0,
                                    0,
                                    benchmark::anymal::zerogravity::params.H,
                                    1.0, 0.0, 0.0, 0.0,
                                    0.03, 0.4, -0.8,
                                    0.03, -0.4, +0.8,
                                    -0.03, 0.4, -0.8,
                                    -0.03, -0.4, 0.8});
  anymal->setInternalCollision(true);
  anymals.push_back(anymal);

  // gravity
  sim->setGravity({0, 0, 0});

  // mass
  benchmark::anymal::zerogravity::params.M = anymal->getTotalMass();
  if(benchmark::anymal::zerogravity::options.gui)
    sim->cameraFollowObject(checkerboard, {10.0, 0.0, 1.0});
}

double simulationLoop(bool timer = true, bool error = true) {
  if(benchmark::anymal::zerogravity::options.gui && benchmark::anymal::zerogravity::options.saveVideo)
    sim->startRecordingVideo("/tmp", "dart-anymal-momentum");

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

    // data save
    if(error) {
      benchmark::anymal::zerogravity::data.ballMomentum.push_back(
          balls[0]->getLinearMomentum()
      );
      benchmark::anymal::zerogravity::data.anymalMomentum.push_back(
          anymals[0]->getLinearMomentumInCartesianSpace()
      );
    }

    // step
    sim->integrate();
  }

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::zerogravity::addDescToOption(desc);
  benchmark::dart::addDescToOption(desc);

  benchmark::anymal::zerogravity::getOptionsFromArg(argc, argv, desc);
  benchmark::dart::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::zerogravity::getParamsFromYAML(benchmark::anymal::zerogravity::getYamlpath().c_str(),
                                                    benchmark::DART);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: DART" << std::endl
                << "GUI      : " << benchmark::anymal::zerogravity::options.gui << std::endl
                << "Solver   : " << benchmark::dart::options.solverOption << std::endl
                << "Timestep : " << benchmark::anymal::zerogravity::options.dt << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::anymal::zerogravity::data.computeError();

  // reset
  balls.clear();
  anymals.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::anymal::zerogravity::options.csv)
    benchmark::anymal::zerogravity::printCSV(benchmark::anymal::zerogravity::getCSVpath(),
                                             benchmark::dart::options.simName,
                                             benchmark::dart::options.solverName,
                                             benchmark::dart::options.detectorName,
                                             benchmark::dart::options.integratorName,
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
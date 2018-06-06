//
// Created by kangd on 14.05.18.
//

#include <BtMbSim.hpp>

#include "AnymalMomentumBenchmark.hpp"
#include "BtMbBenchmark.hpp"

bullet_mb_sim::BtMbSim *sim;
std::vector<bullet_mb_sim::ArticulatedSystemHandle> balls;
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

void setupWorld() {
// add objects
  auto checkerboard = sim->addArticulatedSystem(
      benchmark::anymal::zerogravity::getBulletPlanePath(),
      bullet_mb_sim::object::URDF
  );

  // ball
  auto ball = sim->addArticulatedSystem(
      benchmark::anymal::zerogravity::getBulletBallPath(),
      bullet_mb_sim::object::URDF
  );
  ball->setGeneralizedCoordinate({0,
                                  benchmark::anymal::zerogravity::params.x0,
                                  benchmark::anymal::zerogravity::params.H,
                                  1, 0, 0, 0});
  ball->setGeneralizedVelocity({0,
                                benchmark::anymal::zerogravity::params.v0,
                                0,
                                0, 0, 0});
  balls.push_back(ball);

  // anymal
  auto anymal = sim->addArticulatedSystem(
      benchmark::anymal::zerogravity::getBulletANYmalPath(),
      bullet_mb_sim::object::URDF
  );
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

    // data save
    if(error) {
      benchmark::anymal::zerogravity::data.ballMomentum.push_back(
          balls[0]->getLinearMomentumInCartesianSpace()
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
                                             benchmark::bulletmultibody::options.simName,
                                             benchmark::bulletmultibody::options.solverName,
                                             benchmark::bulletmultibody::options.detectorName,
                                             benchmark::bulletmultibody::options.integratorName,
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
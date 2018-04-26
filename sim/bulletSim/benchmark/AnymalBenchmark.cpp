//
// Created by kangd on 26.04.18.
//

#include <BtWorld_RG.hpp>

#include "AnymalBenchmark.hpp"
#include "raiCommon/utils/StopWatch.hpp"

bullet_sim::BtWorld_RG *sim;
std::vector<bullet_sim::ArticulatedSystemHandle> anymals;

void setupSimulation() {
  if(benchmark::anymal::options.gui)
    sim = new bullet_sim::BtWorld_RG(800, 600, 0.5, benchmark::NO_BACKGROUND, bullet_sim::SOLVER_MULTI_BODY);
  else
    sim = new bullet_sim::BtWorld_RG(bullet_sim::SOLVER_MULTI_BODY);
}

void setupWorld() {
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0.8);

  for(int i = 0; i < benchmark::anymal::options.numRow; i++) {
    for(int j = 0; j < benchmark::anymal::options.numRow; j++) {
      auto anymal = sim->addArticulatedSystem(
          benchmark::anymal::getURDFpath()
      );
      anymal->setColor({1, 0, 0, 1});
      anymal->setGeneralizedCoordinate(
          {i * 2,
           j * 2,
           benchmark::anymal::params.H,
           1.0, 0.0, 0.0, 0.0,
           0.03, 0.4, -0.8,
           -0.03, 0.4, -0.8,
           0.03, -0.4, 0.8,
           -0.03, -0.4, 0.8});
      anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
      anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
      anymals.push_back(anymal);
    }
  }

  sim->setGravity({0, 0, -9.8});

  if(benchmark::anymal::options.gui)
    sim->cameraFollowObject(checkerboard, {1.0, 1.0, 1.0});
}

void simulationLoop() {
  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = benchmark::anymal::params.kp;
  const double kd = benchmark::anymal::params.kd;

  jointNominalConfig << 0, 0, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8,
      -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8,
      -0.03, -0.4, 0.8;

  if(benchmark::anymal::options.gui) {
    // gui
    while(sim->visualizerLoop(0.005, 1.0)) {
      for(int i = 0; i < anymals.size(); i++) {
        jointState = anymals[i]->getGeneralizedCoordinate();
        jointVel = anymals[i]->getGeneralizedVelocity();
        jointForce = anymals[i]->getGeneralizedForce();

        jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
        jointForce.head(6).setZero();
        anymals[i]->setGeneralizedForce(jointForce);
      }
      sim->integrate(0.005);
    }
  } else {
    // no gui
    StopWatch watch;
    watch.start();
    for(int t = 0; t < 50000; t++) {
      for(int i = 0; i < anymals.size(); i++) {
        jointState = anymals[i]->getGeneralizedCoordinate();
        jointVel = anymals[i]->getGeneralizedVelocity();
        jointForce = anymals[i]->getGeneralizedForce();

        jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
        jointForce.head(6).setZero();
        anymals[i]->setGeneralizedForce(jointForce);
      }
      sim->integrate(0.005);
    }

    std::cout<<"time taken for 50k steps "<< watch.measure()<<"s \n";
  }
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::getParamsFromArg(argc, argv);

  setupSimulation();
  setupWorld();
  simulationLoop();

  return 0;
}
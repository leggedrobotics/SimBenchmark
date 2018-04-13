//
// Created by kangd on 04.04.18.
//

#include <DartWorld_RG.hpp>
#include "raiCommon/utils/StopWatch.hpp"

//#define SIM_TIME_MODE

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/ANYmal/robot.urdf";

#ifdef SIM_TIME_MODE
  dart_sim::DartWorld_RG sim;
#else
  dart_sim::DartWorld_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND);
#endif

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  auto anymal = sim.addArticulatedSystem(urdfPath);
  anymal->setGeneralizedCoordinate(
      {0, 0, 1.0,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
  anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

//  sim.setGravity({0, 0, 0});

  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = 40.0, kd = 1.0;

  jointNominalConfig << 0, 0, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

  sim.setTimeStep(0.005);
#ifdef SIM_TIME_MODE
  StopWatch watch;
  watch.start();
  for(int i = 0; i < 50000; i++) {
#else
    sim.cameraFollowObject(checkerboard, {10, 10, 15});
  while(sim.visualizerLoop(0.005, 1.0)) {
#endif
    jointState = anymal->getGeneralizedCoordinate();
    jointVel = anymal->getGeneralizedVelocity();
    jointForce = anymal->getGeneralizedForce();

    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
    jointForce.head(6).setZero();
    anymal->setGeneralizedForce(jointForce);
    sim.integrate();
  }

#ifdef SIM_TIME_MODE
  std::cout << "time taken for 50k steps " << watch.measure() << "s \n";
#endif

  return 0;
}


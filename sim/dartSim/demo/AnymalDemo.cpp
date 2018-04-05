//
// Created by kangd on 04.04.18.
//

#include <World_RG.hpp>
#include "raiCommon/utils/StopWatch.hpp"

#define SIM_TIME_MODE

int main() {

#ifdef SIM_TIME_MODE
  dart_sim::World_RG sim;
#else
  bullet_sim::World_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND, bullet_sim::SOLVER_MULTI_BODY);
#endif

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, 1, -1);
  auto anymal = sim.addArticulatedSystem("../../../res/ANYmal/robot.urdf");
  anymal->setGeneralizedCoordinate(
      {5, 5, 0.6,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
  anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  sim.setGravity({0, 0, -9.8});

  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = 40.0, kd = 1.0;

  jointNominalConfig << 5, 5, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

#ifdef SIM_TIME_MODE
  StopWatch watch;
  watch.start();
  for(int i = 0; i < 10000; i++) {
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
    sim.integrate(0.005);
  }

#ifdef SIM_TIME_MODE
  std::cout<<"time taken for 10k steps "<< watch.measure()<<"s \n";
#endif

  return 0;
}


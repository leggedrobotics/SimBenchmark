//
// Created by kangd on 15.04.18.
//

#include <OdeWorld_RG.hpp>
#include "raiCommon/utils/StopWatch.hpp"

//#define SIM_TIME_MODE
#define VIDEO_SAVE_MODE

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/ANYmal/";

#ifdef SIM_TIME_MODE
  ode_sim::OdeWorld_RG sim;
#else
  ode_sim::OdeWorld_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND);
#endif

  /// NOTE erp should be set to 0.2 for articulated system simulation on ODE
  sim.setERP(0.2, 0.2, 0.2);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);
  auto anymal = sim.addArticulatedSystem(urdfPath);

  anymal->setGeneralizedCoordinate(
      {0, 0, 0.54,
       1.0, 0.0, 0.0, 0.0,
       -0.03, -0.4, 0.8,
       0.03, -0.4, 0.8,
       -0.03, 0.4, -0.8,
       0.03, 0.4, -0.8});
//  anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

//  sim.setGravity({0, 0, 0});

  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = 40.0, kd = 1.0;

  jointNominalConfig << 0, 0, 0.54,
      1.0, 0.0, 0.0, 0.0,
      -0.03, -0.4, 0.8,
      0.03, -0.4, 0.8,
      -0.03, 0.4, -0.8,
      0.03, 0.4, -0.8;

#if defined(SIM_TIME_MODE)
  StopWatch watch;
  watch.start();
  for(int i = 0; i < 50000; i++) {
#else
    sim.cameraFollowObject(checkerboard, {1, 1, 1});
#if defined(VIDEO_SAVE_MODE)
  sim.startRecordingVideo("/tmp", "odeAnymal");
  for(int i = 0; i < 2000 && sim.visualizerLoop(0.005, 1.0); i++) {
#else
    while(sim.visualizerLoop(0.005, 0.1)) {
#endif
#endif
    jointState = anymal->getGeneralizedCoordinate();
    jointVel = anymal->getGeneralizedVelocity();
//    jointForce = anymal->getGeneralizedForce();

//      RAIINFO(jointState)
//      RAIINFO(jointVel)
    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
    jointForce.head(6).setZero();
//    anymal->setGeneralizedForce(jointForce);
    sim.integrate(0.005);
  }

#if defined(SIM_TIME_MODE)
  std::cout<<"time taken for 50k steps "<< watch.measure()<<"s \n";
#elif defined(VIDEO_SAVE_MODE)
  sim.stopRecordingVideo();
#endif

  return 0;
}


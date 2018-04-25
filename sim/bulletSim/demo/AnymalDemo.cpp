//
// Created by kangd on 11.02.18.
//

#include <BtWorld_RG.hpp>
#include "raiCommon/utils/StopWatch.hpp"

//#define SIM_TIME_MODE
//#define VIDEO_SAVE_MODE

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/ANYmal/robot.urdf";

#if defined(SIM_TIME_MODE)
  bullet_sim::World_RG sim(bullet_sim::SOLVER_MULTI_BODY);
#else
  bullet_sim::BtWorld_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND, bullet_sim::SOLVER_MULTI_BODY);
#endif

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);

  std::vector<bullet_sim::ArticulatedSystemHandle> anymals;
  int numRow = 1;

  for(int i = 0; i < numRow; i++) {
    for(int j = 0; j < numRow; j++) {
      auto anymal = sim.addArticulatedSystem(urdfPath);
      anymal->setColor({1, 0, 0, 1});
      anymal->setGeneralizedCoordinate(
          {i * 2 - 5 * 0.5, j * 2 - 5 * 0.5, 0.54,
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

  sim.setGravity({0, 0, -9.8});

  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = 40.0, kd = 1.0;

  jointNominalConfig << 0, 0, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

#if defined(SIM_TIME_MODE)
  StopWatch watch;
  watch.start();
  for(int i = 0; i < 50000; i++) {
#else
  sim.cameraFollowObject(checkerboard, {1.0, 1.0, 1.0});
#if defined(VIDEO_SAVE_MODE)
  sim.startRecordingVideo("/tmp", "btAnymal");
  for(int i = 0; i < 2000 && sim.visualizerLoop(0.005, 1.0); i++) {
#else
  while(sim.visualizerLoop(0.005, 1.0)) {
#endif
#endif
    for(int i = 0; i < numRow * numRow; i++) {
      jointState = anymals[i]->getGeneralizedCoordinate();
      jointVel = anymals[i]->getGeneralizedVelocity();
      jointForce = anymals[i]->getGeneralizedForce();

      jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
      jointForce.head(6).setZero();
      anymals[i]->setGeneralizedForce(jointForce);
      sim.integrate(0.005);
    }
  }

#if defined(SIM_TIME_MODE)
  std::cout<<"time taken for 50k steps "<< watch.measure()<<"s \n";
#elif defined(VIDEO_SAVE_MODE)
  sim.stopRecordingVideo();
#endif

  return 0;
}
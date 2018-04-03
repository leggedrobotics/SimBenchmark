//
// Created by kangd on 13.02.18.
//

#include <raiSim/World_RG.hpp>

#define SIM_TIME_MODE

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/ANYmal/";

#ifdef SIM_TIME_MODE
  rai_sim::World_RG raiSim;
#else
  rai_sim::World_RG raiSim(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  raiSim.setLightPosition(30, 0, 10);
#endif
  raiSim.setGravity({0, 0, -9.8});

  // add objects
  auto checkerboard = raiSim.addCheckerboard(2, 100, 100, 0.1, 1, -1, rai_sim::GRID);

  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);

  jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  std::vector<rai_sim::ArticulatedSystemHandle> animals;

  auto anymal = raiSim.addArticulatedSystem(urdfPath);
  anymal->setGeneralizedCoordinate({0, 0, 0.6,
                                    1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
                                    -0.8, -0.03, 0.4, -0.8, 0.03, -0.4,
                                    0.8, -0.03, -0.4, 0.8});
  anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  const double kp = 40.0, kd = 1.0;

#ifdef SIM_TIME_MODE
  StopWatch watch;
  watch.start();
  for(int i = 0; i < 10000; i++) {
#else
  raiSim.cameraFollowObject(checkerboard, {2, 2, 2});
  while(raiSim.visualizerLoop(0.01, 1.0)) {
#endif
    raiSim.integrate1(0.01);

//    jointState = anymal->getGeneralizedCoordinate();
//    jointVel = anymal->getGeneralizedVelocity();
//    jointForce = anymal->getGeneralizedForce();
//
//    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
//    jointForce.head(6).setZero();
    anymal->setGeneralizedForce(jointForce);

    raiSim.integrate2(0.01);
  }

#ifdef SIM_TIME_MODE
  std::cout<<"time taken for 100k steps "<< watch.measure()<<"s \n";
#endif

  return 0;
}
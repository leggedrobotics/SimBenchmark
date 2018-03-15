//
// Created by kangd on 13.02.18.
//

#include <raiSim/World_RG.hpp>

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/ANYmal/";

  rai_sim::World_RG raiSim(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  raiSim.setGravity({0, 0, -9.8});
  raiSim.setLightPosition(30, 0, 10);

  // add objects
  auto checkerboard = raiSim.addCheckerboard(2, 100, 100, 0.1, 1, -1, rai_sim::GRID);

  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);

  jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  std::vector<rai_sim::ArticulatedSystemHandle> animals;

  auto anymal = raiSim.addArticulatedSystem(urdfPath);
  anymal->setGeneralizedCoordinate({0, 0, 0.54,
                                    1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
                                    -0.8, -0.03, 0.4, -0.8, 0.03, -0.4,
                                    0.8, -0.03, -0.4, 0.8});
  anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  const double kp = 40.0, kd = 1.0;

  raiSim.cameraFollowObject(checkerboard, {2, 2, 2});
  StopWatch watch;

  watch.start();
  while(raiSim.visualizerLoop(0.01, 1.0)) {
    raiSim.integrate1(0.01);

    jointState = anymal->getGeneralizedCoordinate();
    jointVel = anymal->getGeneralizedVelocity();
    jointForce = anymal->getGeneralizedForce();

    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
    jointForce.head(6).setZero();
    anymal->setGeneralizedForce(jointForce);

    raiSim.integrate2(0.01);
  }
  std::cout<<"time taken for 100k steps "<<watch.measure()<<"s \n";
  return 0;
}
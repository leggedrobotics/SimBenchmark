//
// Created by kangd on 16.02.18.
//

#include <raiSim/World_RG.hpp>

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../res/ANYmal/";

  rai_sim::World_RG raiSim(800, 600, 0.5);
  raiSim.setGravity({0, 0, 0});
  raiSim.setLightPosition(30, 0, 10);

  // add objects
  auto anymal = raiSim.addArticulatedSystem(urdfPath);
  anymal->setGeneralizedCoordinate({0, 0, 0.54,
                                    1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
                                    -0.8, -0.03, 0.4, -0.8, 0.03, -0.4,
                                    0.8, -0.03, -0.4, 0.8});
  anymal->setGeneralizedVelocity({0, 2, 0,
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0});
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  auto anymal2 = raiSim.addArticulatedSystem(urdfPath);
  anymal2->setGeneralizedCoordinate({0, 5, 0.54,
                                    1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
                                    -0.8, -0.03, 0.4, -0.8, 0.03, -0.4,
                                    0.8, -0.03, -0.4, 0.8});
  anymal2->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal2->getDOF()));
  anymal2->setGeneralizedForce(Eigen::VectorXd::Zero(anymal2->getDOF()));

  raiSim.cameraFollowObject(anymal, {2, 2, 2});
  StopWatch watch;

  watch.start();
  while(raiSim.visualizerLoop(0.01, 1.0)) {
    raiSim.integrate1(0.01);
    raiSim.integrate2(0.01);
  }
  std::cout<<"time taken for 100k steps "<<watch.measure()<<"s \n";
  return 0;
}
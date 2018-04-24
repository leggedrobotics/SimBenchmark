//
// Created by kangd on 24.04.18.
//

#include <raiSim/World_RG.hpp>

int main() {

  // timer
  std::string path = "/tmp";
  rai::Utils::timer->setLogPath(path);
  rai::Utils::logger->setLogPath(path);

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/2DRobotArm/";

  rai_sim::World_RG sim(800, 600, 0.5, rai_sim::NO_BACKGROUND);

  auto arm = sim.addArticulatedSystem(urdfPath);
  auto checkerBoard = sim.addCheckerboard(1, 100, 100, 0.1,
                                          1, -1,
                                          rai_sim::GRID);

  Eigen::VectorXd gc(arm->getStateDim());
  Eigen::VectorXd gv(arm->getDOF()), tau(arm->getDOF());

  gc.setZero();
  gc(0) = 0.1;
  gv.setZero();
  tau.setZero();

  arm->setState(gc, gv);
  arm->setGeneralizedForce(tau);

  sim.cameraFollowObject(checkerBoard, {0.5, 0.5, 0.5});
  sim.loop(0.001, 1);
}
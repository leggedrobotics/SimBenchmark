//
// Created by kangd on 16.05.18.
//

#include <BtSim.hpp>

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Singlebody/";

  bullet_sim::BtSim sim(800, 600, 0.5, bullet_sim::SOLVER_MULTI_BODY, benchmark::NO_BACKGROUND);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  auto robot = sim.addArticulatedSystem(urdfPath);
  robot->setColor({1, 0, 0, 1});
  robot->setGeneralizedCoordinate(
      {0, 0, 0.25,
       1, 0, 0, 0});
  robot->setGeneralizedVelocity(Eigen::VectorXd::Zero(robot->getDOF()));
  robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));

  sim.setGravity({0, 0, -9.8});
  sim.cameraFollowObject(checkerboard, {1.0, 1.0, 1.0});

  for(int i = 0; i < 2000 && sim.visualizerLoop(0.005, 1.0); i++) {
    sim.integrate(0.005);
  }

  return 0;
}
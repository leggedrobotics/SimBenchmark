//
// Created by kangd on 26.05.18.
//

#include "BtMbSim.hpp"

int main(int argc, const char* argv[]) {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Singlebody/";

  bullet_mb_sim::BtMbSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  sim.setGravity({0, 0, -9.81});
  sim.setERP(0.2, 0.2, 0.2);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  sim.cameraFollowObject(checkerboard, {5, 0, 5});

  auto robot = sim.addArticulatedSystem(urdfPath, bullet_mb_sim::object::URDF);
  robot->setGeneralizedCoordinate({0, 2, 4,
                                   1, 0, 0, 0});

  while(sim.visualizerLoop(0.005, 1.0)) {
    sim.integrate();
  }
  return 0;
}
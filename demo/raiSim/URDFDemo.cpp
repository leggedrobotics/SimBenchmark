//
// Created by kangd on 15.04.18.
//

#include <raiSim/World_RG.hpp>
#include "raiCommon/utils/StopWatch.hpp"

//#define VIDEO_SAVE_MODE

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Singlebody/robot.urdf";

  rai_sim::World_RG sim(800, 600, 0.5, rai_sim::NO_BACKGROUND);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, 1, -1, rai_sim::GRID);
  sim.cameraFollowObject(checkerboard, {5, 0, 5});

  auto robot = sim.addArticulatedSystem(urdfPath);
  robot->setGeneralizedCoordinate({0, 2, 4,
                                   1, 0, 0, 0});
  auto robot2 = sim.addArticulatedSystem(urdfPath);
  robot2->setGeneralizedCoordinate({0, 2, 8,
                                   1, 0, 0, 0});

  while(sim.visualizerLoop(0.005, 1.0)) {
    sim.integrate(0.005);
  }

  return 0;
}


//
// Created by kangd on 15.04.18.
//

#include <OdeSim.hpp>
#include "raiCommon/utils/StopWatch.hpp"

//#define VIDEO_SAVE_MODE

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Multibody/";

  ode_sim::OdeSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  sim.setERP(0.2, 0, 0);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0.8);
  sim.cameraFollowObject(checkerboard, {5, 0, 5});

  auto robot = sim.addArticulatedSystem(urdfPath);
  robot->setGeneralizedCoordinate({0, 0, 0.5,
                                   1, 0, 0, 0, 
                                   0});

  while(sim.visualizerLoop(0.005, 1.0)) {
    sim.integrate(0.005);
  }

  return 0;
}


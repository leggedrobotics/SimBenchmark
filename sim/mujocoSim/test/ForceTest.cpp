//
// Created by kangd on 12.05.18.
//

#include "MjcSim.hpp"

int main() {

  // file path
  std::string xmlPath(__FILE__);
  while (xmlPath.back() != '/')
    xmlPath.erase(xmlPath.size() - 1, 1);
  xmlPath += "../../../res/mujoco/test/forcetest.xml";

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../../../lib/mjpro150/mjkey.txt";

  // load model from file and check for errors
  mujoco_sim::MjcSim sim(800,
                              600,
                              0.5,
                              xmlPath.c_str(),
                              keyPath.c_str(),
                              benchmark::NO_BACKGROUND);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {10, 0, 20});

  // gravity
  double g = -9.8;
  sim.setGravity({0, 0, g});

  // run simulation for 10 seconds
  sim.setTimeStep(0.01);

  while(sim.visualizerLoop(0.01)) {
    sim.integrate1();
    sim.getSingleBodyHandle(1)->setExternalForce({0, 0, -10 * g});
    sim.integrate2();
  }

  RAIINFO(sim.getSingleBodyHandle(1)->getLinearVelocity())

  return 0;
}

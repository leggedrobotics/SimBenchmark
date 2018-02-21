//
// Created by kangd on 18.02.18.
//

#include <mujoco.h>
#include <mujocoSim/World_RG.hpp>

#include "bounce.hpp"

int main() {
  // logger
  std::string path = benchmark::parentDir + "mujoco";
  std::string name = std::to_string(benchmark::dt);
  rai::Utils::logger->setOptions(rai::Utils::ONEFILE_FOR_ONEDATA);
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->setLogFileName(name);
  rai::Utils::logger->addVariableToLog(3, "vel_ball", "linear velocity of ball");
  rai::Utils::logger->addVariableToLog(3, "pos_ball", "position of ball");

  // load model from file and check for errors
  mujoco_sim::World_RG sim(800, 600, 0.5, "/home/kangd/git/benchmark/benchmark/mujoco/test.xml", benchmark::NO_BACKGROUND);
  sim.setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {10, 0, 5});

  // run simulation for 10 seconds
  sim.loop(0.01, 1.0);

//  while( world.getWorldData()->time<10 ) {
//    rai::Utils::logger->appendData("vel_ball", world.getObjectList()[1]->getLinearVelocity().data());
//    rai::Utils::logger->appendData("pos_ball", world.getObjectList()[1]->getPosition().data());
//  }

  return 0;
}

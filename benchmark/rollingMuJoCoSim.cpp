//
// Created by kangd on 21.02.18.
//

#include <mujocoSim/World_RG.hpp>

#include "rolling.hpp"

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
  mujoco_sim::World_RG sim(800, 600, 0.5, "/home/kangd/git/benchmark/benchmark/mujoco/rolling.xml", benchmark::NO_BACKGROUND);
  sim.setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {30, 0, 10});

// simulation loop
  // press 'q' key to quit
  for(int i = 0; i < benchmark::simulationTime / benchmark::dt && sim.visualizerLoop(benchmark::dt); i++) {
    sim.getSingleBodyHandle(1)->setExternalForce(benchmark::force);
    // log
    rai::Utils::logger->appendData("linvel_box", sim.getSingleBodyHandle(1)->getLinearVelocity().data());
    rai::Utils::logger->appendData("linvel_ball", sim.getSingleBodyHandle(2)->getLinearVelocity().data());
    rai::Utils::logger->appendData("pos_box", sim.getSingleBodyHandle(1)->getPosition().data());
    rai::Utils::logger->appendData("pos_ball", sim.getSingleBodyHandle(2)->getPosition().data());
    sim.integrate(benchmark::dt);
  }

  return 0;
}

//
// Created by kangd on 15.02.18.
//

#include <bulletSim/World_RG.hpp>

#include "rolling.hpp"

int main() {

  // logger
  std::string path = "/tmp/bullet_rolling";
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->addVariableToLog(3, "linvel_box", "linear velocity of box");
  rai::Utils::logger->addVariableToLog(3, "linvel_ball", "linear velocity of ball");

  bullet_sim::World_RG sim(800, 600, 0.5, bullet_sim::NO_BACKGROUND);
  sim.setGravity(benchmark::gravity);
  sim.setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);

  // add objects
  auto checkerboard = sim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setFrictionCoefficient(benchmark::groundMu);

  auto box = sim.addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5);
  box->setFrictionCoefficient(benchmark::boxMu);

  std::vector<bullet_sim::SingleBodyHandle> objectList;

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim.addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5);
      ball->setFrictionCoefficient(benchmark::ballMu);
      objectList.push_back(ball);
    }
  }

  // camera relative position
  sim.cameraFollowObject(checkerboard, {10, 0, 15});

  // iteration counter
  int cnt = 0;

  // simulation loop
  // press 'q' key to quit
  while (sim.visualizerLoop(benchmark::dt, 1)) {
    if (cnt++ < benchmark::simulationTime / benchmark::dt) {
      box->setExternalForce(benchmark::force);

      // log
      rai::Utils::logger->appendData("linvel_box", box->getLinearVelocity().data());
      rai::Utils::logger->appendData("linvel_ball", objectList[0]->getLinearVelocity().data());
    }

    sim.integrate(benchmark::dt);
  }
  return 0;
}

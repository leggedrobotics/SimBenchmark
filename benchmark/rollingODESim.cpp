//
// Created by kangd on 15.02.18.
//

#include "odeSim/World_RG.hpp"

#include "rolling.hpp"

int main() {

  // logger
  std::string path = benchmark::dataPath + benchmark::parentDir + "ode";
  std::string name = std::to_string(benchmark::dt);
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->setLogFileName(name);
  rai::Utils::logger->setOptions(rai::Utils::ONEFILE_FOR_ONEDATA);
  rai::Utils::logger->addVariableToLog(3, "linvel_box", "linear velocity of box");
  rai::Utils::logger->addVariableToLog(3, "linvel_ball", "linear velocity of ball");
  rai::Utils::logger->addVariableToLog(3, "pos_box", "position of box");
  rai::Utils::logger->addVariableToLog(3, "pos_ball", "position of ball");

  // timer
  std::string timer = name + "timer";
  rai::Utils::timer->setLogPath(path);
  rai::Utils::timer->setLogFileName(timer);

  // sim
  ode_sim::World_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND, ode_sim::SOLVER_STANDARD);
  sim.setGravity(benchmark::gravity);
  sim.setERP(benchmark::erp, 0, 0);
  sim.setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);

  // add objects
  auto checkerboard = sim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setFrictionCoefficient(benchmark::groundMu);

  auto box = sim.addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5);
  box->setFrictionCoefficient(benchmark::boxMu);

  std::vector<benchmark::SingleBodyHandle> objectList;

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim.addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5);
      ball->setFrictionCoefficient(benchmark::ballMu);
      objectList.push_back(ball);
    }
  }

  // camera relative position
  sim.cameraFollowObject(checkerboard, {30, 0, 10});

  // simulation loop
  // press 'q' key to quit
  rai::Utils::timer->startTimer("rolling");
  for(int i = 0; i < benchmark::simulationTime / benchmark::dt && sim.visualizerLoop(benchmark::dt); i++) {
    box->setExternalForce(benchmark::force);
    // log
    rai::Utils::logger->appendData("linvel_box", box->getLinearVelocity().data());
    rai::Utils::logger->appendData("linvel_ball", objectList[0]->getLinearVelocity().data());
    rai::Utils::logger->appendData("pos_box", box->getPosition().data());
    rai::Utils::logger->appendData("pos_ball", objectList[0]->getPosition().data());
    sim.integrate(benchmark::dt);
  }
  rai::Utils::timer->stopTimer("rolling");

  return 0;
}
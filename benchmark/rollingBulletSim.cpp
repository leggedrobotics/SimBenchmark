//
// Created by kangd on 15.02.18.
//

#include <bulletSim/World_RG.hpp>

int main() {

  // parameters
  double dt = 0.001;                     // time step
  const int forceMaxTime = 3;           // time for applying force
  Eigen::Vector3d force = {0, 150, 0};  // force

  // logger
  std::string path = "/tmp";
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->addVariableToLog(3, "linvel_box", "linear velocity of box");
  rai::Utils::logger->addVariableToLog(3, "linvel_ball", "linear velocity of ball");

  bullet_sim::World_RG sim(800, 600, 0.5, bullet_sim::NO_BACKGROUND);
  sim.setGravity({0, 0, -9.8});
  sim.setLightPosition(30, 0, 10);

  // add objects
  auto checkerboard = sim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setFrictionCoefficient(0.5);

  auto box = sim.addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5);
  box->setFrictionCoefficient(0.8);

  std::vector<bullet_sim::SingleBodyHandle> objectList;

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim.addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5);
      ball->setFrictionCoefficient(1.0);
      objectList.push_back(ball);
    }
  }

  // camera relative position
  sim.cameraFollowObject(checkerboard, {10, 0, 15});

  // iteration counter
  int cnt = 0;

  // simulation loop
  // press 'q' key to quit
  while (sim.visualizerLoop(dt, 1)) {
    if (++cnt < forceMaxTime / dt) {
      box->setExternalForce(force);

      // log
      rai::Utils::logger->appendData("linvel_box", box->getLinearVelocity().data());
      rai::Utils::logger->appendData("linvel_ball", objectList[0]->getLinearVelocity().data());
    }

    sim.integrate(dt);
  }
  return 0;
}

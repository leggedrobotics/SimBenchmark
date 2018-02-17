//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

#include "bounce.hpp"

int main() {

  // logger
  std::string path = "/tmp/rai_bounce";
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->addVariableToLog(3, "vel_ball", "linear velocity of ball");
  rai::Utils::logger->addVariableToLog(3, "pos_ball", "position of ball");

  rai_sim::World_RG sim(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  rai_sim::MaterialManager materials;

  sim.setGravity(benchmark::gravity);
  sim.setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);

  // materials
  materials.setMaterialNames({"ground", "ball"});
  materials.setMaterialPairProp("ground", "ball",
                                benchmark::friction,
                                benchmark::restitution,
                                0.01);
  sim.updateMaterialProp(materials);

  // add objects
  // checkerboard
  auto checkerboard = sim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setMaterial(sim.getMaterialKey("ground"));

  // ball
  auto ball = sim.addSphere(benchmark::ballR, benchmark::ballM);
  ball->setMaterial(sim.getMaterialKey("ball"));
  ball->setPosition(0, 0, benchmark::dropHeight);

  // camera relative position
  sim.cameraFollowObject(ball, {10, 0, 5});

  // iteration counter
  int cnt = 0;

  // simulation loop
  // press 'q' key to quit
  while (sim.visualizerLoop(benchmark::dt, 1)) {
    if (cnt++ < benchmark::simulationTime / benchmark::dt) {

      // log
      rai::Utils::logger->appendData("vel_ball", ball->getLinearVelocity().data());
      rai::Utils::logger->appendData("pos_ball", ball->getPosition().data());
    }

    sim.integrate(benchmark::dt);
  }
  return 0;
}
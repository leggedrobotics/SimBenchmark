//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

#include "rolling.hpp"

int main() {

  // logger
  std::string path = benchmark::dataPath + benchmark::parentDir + "rai";
  std::string name = std::to_string(benchmark::dt);
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->setLogFileName(name);
  rai::Utils::logger->setOptions(rai::Utils::ONEFILE_FOR_ONEDATA);
  rai::Utils::logger->addVariableToLog(3, "linvel_box", "linear velocity of box");
  rai::Utils::logger->addVariableToLog(3, "linvel_ball", "linear velocity of ball");
  rai::Utils::logger->addVariableToLog(3, "pos_box", "position of box");
  rai::Utils::logger->addVariableToLog(3, "pos_ball", "position of ball");

  // simulator
  rai_sim::World_RG sim(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  rai_sim::MaterialManager materials;

  sim.setGravity(benchmark::gravity);
  sim.setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);

  // materials
  materials.setMaterialNames({"ground", "box", "ball"});
  materials.setMaterialPairProp("ground", "ball", benchmark::groundMu * benchmark::ballMu, 0.0, 0.01);
  materials.setMaterialPairProp("ground", "box", benchmark::groundMu * benchmark::boxMu, 0.0, 0.01);
  materials.setMaterialPairProp("ball", "box", benchmark::ballMu * benchmark::boxMu, 0.0, 0.01);
  sim.updateMaterialProp(materials);

  // add objects
  auto checkerboard = sim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setMaterial(sim.getMaterialKey("ground"));

  auto box = sim.addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5);
  box->setMaterial(sim.getMaterialKey("box"));

  std::vector<rai_sim::SingleBodyHandle> objectList;

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim.addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5);
      ball->setMaterial(sim.getMaterialKey("ball"));
      objectList.push_back(ball);
    }
  }

  // camera relative position
  sim.cameraFollowObject(box, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  rai_sim::Vec<3> force = {benchmark::force[0], benchmark::force[1], benchmark::force[2]};

  for(int i = 0; i < benchmark::simulationTime / benchmark::dt && sim.visualizerLoop(benchmark::dt); i++) {
    box->setExternalForce(force, 0);
    // log
    rai::Utils::logger->appendData("linvel_box", box->getLinearVelocity().data());
    rai::Utils::logger->appendData("linvel_ball", objectList[0]->getLinearVelocity().data());
    rai::Utils::logger->appendData("pos_box", box->getPosition().data());
    rai::Utils::logger->appendData("pos_ball", objectList[0]->getPosition().data());
    sim.integrate(benchmark::dt);
  }
  return 0;
}
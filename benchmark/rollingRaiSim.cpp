//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

int main() {

  // parameters
  double dt = 0.001;                     // time step
  const int forceMaxTime = 3;           // time for applying force
  rai_sim::Vec<3> force = {0, 150, 0};  // force

  // logger
  std::string path = "/tmp";
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->addVariableToLog(3, "linvel_box", "linear velocity of box");
  rai::Utils::logger->addVariableToLog(3, "linvel_ball", "linear velocity of ball");

  // simulator
  rai_sim::World_RG sim(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  rai_sim::MaterialManager materials;

  sim.setGravity({0, 0, -9.8});
  sim.setLightPosition(30, 0, 10);

  // materials
  materials.setMaterialNames({"ground", "box", "ball"});
  materials.setMaterialPairProp("ground", "ball", 0.5, 0.0, 0.01);
  materials.setMaterialPairProp("ground", "box", 0.4, 0.0, 0.01);
  materials.setMaterialPairProp("ball", "box", 0.8, 0.0, 0.01);
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

  // iteration counter
  int cnt = 0;

  // simulation loop
  // press 'q' key to quit
  while (sim.visualizerLoop(dt, 1)) {
    if (++cnt < forceMaxTime / dt) {
      box->setExternalForce(force, 0);

      // log
      rai::Utils::logger->appendData("linvel_box", box->getLinearVelocity().data());
      rai::Utils::logger->appendData("linvel_ball", objectList[0]->getLinearVelocity().data());
    }

    sim.integrate(dt);
  }
  return 0;
}
//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

int main() {

  rai_sim::World_RG raiSim(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  rai_sim::MaterialManager materials;

  raiSim.setGravity({0, 0, -9.8});
  raiSim.setLightPosition(30, 0, 10);

  // materials
  materials.setMaterialNames({"ground", "box", "ball"});
  materials.setMaterialPairProp("ground", "ball", 0.5, 0.0, 0.01);
  materials.setMaterialPairProp("ground", "box", 0.4, 0.0, 0.01);
  materials.setMaterialPairProp("ball", "box", 0.8, 0.0, 0.01);
  raiSim.updateMaterialProp(materials);

  // add objects
  auto checkerboard = raiSim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setMaterial(raiSim.getMaterialKey("ground"));

  auto box1 = raiSim.addBox(20, 20, 1, 1);
  box1->setPosition(0, 0, 0.5);
  box1->setMaterial(raiSim.getMaterialKey("box"));

  auto ball1 = raiSim.addSphere(0.5, 1);
  ball1->setPosition(0, -5, 1.5);
  ball1->setMaterial(raiSim.getMaterialKey("ball"));
  auto ball2 = raiSim.addSphere(0.5, 1);
  ball2->setPosition(0, 0, 1.5);
  ball2->setMaterial(raiSim.getMaterialKey("ball"));
  auto ball3 = raiSim.addSphere(0.5, 1);
  ball3->setPosition(0, 5, 1.5);
  ball3->setMaterial(raiSim.getMaterialKey("ball"));

  // timestep
  double dt = 0.01;

  // camera relative position
  raiSim.cameraFollowObject(box1, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  while (raiSim.visualizerLoop(dt, 1)) {
    rai_sim::Vec<3> force = {0, 10, 0};
    box1->setExternalForce(force, 0);
    raiSim.integrate(dt);
  }

  return 0;
}
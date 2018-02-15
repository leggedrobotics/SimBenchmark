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
  materials.setMaterialNames({"ground", "ball"});
  materials.setMaterialPairProp("ground", "ball", 0.8, 1.0, 0.01);
  raiSim.updateMaterialProp(materials);

  // add objects
  // checkerboard
  auto checkerboard = raiSim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setMaterial(raiSim.getMaterialKey("ground"));

  // ball
  auto ball = raiSim.addSphere(0.5, 1);
  ball->setMaterial(raiSim.getMaterialKey("ball"));
  ball->setPosition(0, 0, 10);

  // timestep
  double dt = 0.01;

  // camera relative position
  raiSim.cameraFollowObject(ball, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  raiSim.loop(dt, 1.0);

  return 0;
}
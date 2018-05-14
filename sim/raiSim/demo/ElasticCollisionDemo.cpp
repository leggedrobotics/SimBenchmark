//
// Created by kangd on 14.05.18.
//

#include <raiSim/World_RG.hpp>

enum Object {
  BOX,
  BALL,
  CAPSULE
};

int main() {

  rai_sim::World_RG sim(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  sim.setGravity({0, 0, 0});
  sim.setLightPosition(30, 0, 10);

  rai_sim::MaterialManager materials;
  materials.setMaterialNames({"ground", "ball"});
  materials.setMaterialPairProp("ground", "ball",
                                0, 1, 0);
  materials.setMaterialPairProp("ball", "ball",
                                0, 1, 0);
  sim.updateMaterialProp(materials);

  // add objects
  auto checkerboard = sim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setMaterial(sim.getMaterialKey("ground"));

  auto ball1 = sim.addSphere(0.5, 1);
  ball1->setPosition(0, 5, 2);
  ball1->setVelocity(0,-1,0,0,0,0);
  ball1->setMaterial(sim.getMaterialKey("ball"));
  auto ball2 = sim.addSphere(0.5, 1);
  ball2->setPosition(0, -5, 2);
  ball2->setVelocity(0,1,0,0,0,0);
  ball2->setMaterial(sim.getMaterialKey("ball"));

  // timestep
  double dt = 0.01;

  // camera relative position
  sim.cameraFollowObject(checkerboard, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  sim.loop(dt, 1.0);

  RAIINFO("ball1 vel = " << ball1->getLinearVelocity())
  RAIINFO("ball2 vel = " << ball2->getLinearVelocity())

  return 0;
}
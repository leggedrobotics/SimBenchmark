//
// Created by kangd on 11.02.18.
//

#include "bulletSim/World_RG.hpp"

int main() {

  bullet_sim::World_RG bulletSim(800, 600, 0.5);
  bulletSim.setGravity({0, 0, -9.8});
  bulletSim.setLightPosition(30, 0, 10);

  // add objects
  auto checkerboard = bulletSim.addCheckerboard(5.0, 100.0, 100.0, 0.1);

  auto box1 = bulletSim.addBox(1, 1, 1, 100);
  box1->setPosition(0, 0, 10.0);

  double dt = 0.01;  // (sec)

  // camera relative position
  bulletSim.cameraFollowObject(box1, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  bulletSim.loop(dt, 1.0);

  return 0;
}
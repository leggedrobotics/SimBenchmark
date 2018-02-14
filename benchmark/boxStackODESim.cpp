//
// Created by kangd on 12.02.18.
//

#include "odeSim/World_RG.hpp"

int main() {

  ode_sim::World_RG odeSim(800, 600, 0.5, ode_sim::NO_BACKGROUND);
  odeSim.setGravity({0, 0, -9.8});
  odeSim.setLightPosition(30, 0, 10);

  // add objects
  auto checkerboard = odeSim.addCheckerboard(5.0, 100.0, 100.0, 0.1);

  auto box1 = odeSim.addBox(1, 1, 1, 100);
  box1->setPosition(0, 0, 0.5);

  double dt = 0.01;  // (sec)

  // camera relative position
  odeSim.cameraFollowObject(box1, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  odeSim.loop(dt, 1.0);

  return 0;
}
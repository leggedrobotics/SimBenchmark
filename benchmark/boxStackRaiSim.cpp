//
// Created by kangd on 09.02.18.
//

#include <raiSim/World_RG.hpp>

int main() {

  rai_sim::World_RG raiSim(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  raiSim.setGravity({0, 0, -9.8});
  raiSim.setLightPosition(30, 0, 10);

  // add objects
  auto checkerboard = raiSim.addCheckerboard(5.0, 100.0, 100.0, 0.1);

  auto box1 = raiSim.addBox(1, 1, 1, 100);
  box1->setPosition(0, 0, 0.5);
  auto box2 = raiSim.addBox(1, 1, 1, 100);
  box2->setPosition(0, 0, 1.5);
  auto box3 = raiSim.addBox(1, 1, 1, 100);
  box3->setPosition(0, 0, 2.5);
  auto box4 = raiSim.addBox(1, 1, 1, 100);
  box4->setPosition(0, 0, 3.5);

  // timestep
  double dt = 0.01;

  // camera relative position
  raiSim.cameraFollowObject(box1, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  raiSim.loop(dt, 1.0);

  return 0;
}
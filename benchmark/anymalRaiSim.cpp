//
// Created by kangd on 13.02.18.
//

#include <raiSim/World_RG.hpp>

int main() {

  rai_sim::World_RG raiSim(800, 600, 0.5);
  raiSim.setGravity({0, 0, -9.8});
  raiSim.setLightPosition(30, 0, 10);

  // add objects
  auto checkerboard = raiSim.addCheckerboard(5.0, 100.0, 100.0, 0.1);

  // timestep
  double dt = 0.01;

  // camera relative position
  raiSim.cameraFollowObject(box1, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  raiSim.loop(dt, 1.0);

  return 0;
}
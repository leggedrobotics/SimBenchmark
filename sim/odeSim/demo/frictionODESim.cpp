//
// Created by kangd on 16.02.18.
//

#include <odeSim/World_RG.hpp>

int main() {

  ode_sim::World_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND);

  sim.setGravity({0, 0, -9.8});
  sim.setLightPosition(30, 0, 10);

  // add objects
  // checkerboard
  auto checkerboard = sim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setFrictionCoefficient(1.0);

  // box
  auto box = sim.addBox(1, 1, 1, 10);
  box->setFrictionCoefficient(0.5);
  box->setPosition(0, 0, 0.5);
  box->setVelocity(0, 10, 0, 0, 0, 0);

  // timestep
  double dt = 0.01;

  // camera relative position
  sim.cameraFollowObject(box, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  sim.loop(dt, 1.0);

  RAIINFO(box->getPosition());

  return 0;
}

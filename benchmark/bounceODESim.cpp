//
// Created by kangd on 16.02.18.
//

#include <odeSim/World_RG.hpp>

int main() {

  ode_sim::World_RG sim(800, 600, 0.5, ode_sim::NO_BACKGROUND);

  sim.setGravity({0, 0, -9.8});
  sim.setLightPosition(30, 0, 10);

  // add objects
  // checkerboard
  auto checkerboard = sim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setRestitutionCoefficient(1.0);
  checkerboard->setFrictionCoefficient(0);

  // ball
  auto ball = sim.addSphere(0.5, 1);
  ball->setRestitutionCoefficient(1.0);
  ball->setFrictionCoefficient(0);
  ball->setPosition(0, 0, 10);

  // timestep
  double dt = 0.01;

  // camera relative position
  sim.cameraFollowObject(ball, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  sim.loop(dt, 1.0);

  return 0;
}

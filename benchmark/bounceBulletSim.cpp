//
// Created by kangd on 15.02.18.
//

#include <bulletSim/World_RG.hpp>

int main() {

  bullet_sim::World_RG bulletSim(800, 600, 0.5, bullet_sim::NO_BACKGROUND);

  bulletSim.setGravity({0, 0, -9.8});
  bulletSim.setLightPosition(30, 0, 10);

  // add objects
  // checkerboard
  auto checkerboard = bulletSim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setRestitution(1.0);
  checkerboard->setFriction(0);

  // ball
  auto ball = bulletSim.addSphere(0.5, 1);
  ball->setRestitution(1.0);
  ball->setFriction(0);
  ball->setPosition(0, 0, 10);

  // timestep
  double dt = 0.01;

  // camera relative position
  bulletSim.cameraFollowObject(ball, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  bulletSim.loop(dt, 1.0);

  return 0;
}
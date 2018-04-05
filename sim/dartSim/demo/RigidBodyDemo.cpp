//
// Created by kangd on 05.04.18.
//

#include <World_RG.hpp>

int main() {

  dart_sim::World_RG sim(800, 600, 0.5);

//  sim.setGravity({0,0,0});
  sim.setLightPosition(30, 0, 10);

  // checkerboard
  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, 1, -1);

  // ball
  auto ball = sim.addSphere(0.5, 10.0);
//  ball.visual()[0]->setColor({1.0, 1.0, 0.0});
  ball->setPosition(0.0, 0.0, 10);

  // simulation loop
  // press 'q' key to quit
  double dt = 0.01;  // (sec)
  sim.setTimeStep(dt);
  sim.loop();
}

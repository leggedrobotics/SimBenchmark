//
// Created by kangd on 05.04.18.
//

#include <DartSim.hpp>

int main() {

  dart_sim::DartSim sim(800, 600, 0.5, 0,
                             dart_sim::SOLVER_LCP_PGS,
                             dart_sim::COLLISION_DETECTOR_BULLET);

//  sim.setGravity({0,0,0});
  sim.setLightPosition(30, 0, 10);

  // checkerboard
  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1);

  // ball
  auto ball = sim.addSphere(0.5, 10.0);
  ball.visual()[0]->setColor({1.0, 0.0, 0.0});
  ball->setPosition(0.0, 0.0, 10);

  auto box = sim.addBox(0.5, 0.5, 0.5, 10.0);
  box.visual()[0]->setColor({0.0, 1.0, 0.0});
  box->setPosition(0.0, 5.0, 10);
  box->setOrientation(0.2, 0.3827, 0, 0);

  auto cylinder = sim.addCylinder(0.5, 1.0, 10.0);
  cylinder.visual()[0]->setColor({0.0, 0.0, 1.0});
  cylinder->setPosition(5.0, 0.0, 10);
  sim.cameraFollowObject(checkerboard, {5, 0, 5});

  // simulation loop
  // press 'q' key to quit
  double dt = 0.01;  // (sec)
  sim.setTimeStep(dt);
  sim.loop();
}

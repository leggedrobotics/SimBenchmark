//
// Created by kangd on 15.02.18.
//

#include "odeSim/World_RG.hpp"

int main() {

  // parameters
  double dt = 0.001;                     // time step
  const int forceMaxTime = 3;           // time for applying force
  Eigen::Vector3d force = {0, 150, 0};  // force

  ode_sim::World_RG sim(800, 600, 0.5, ode_sim::NO_BACKGROUND, ode_sim::SOLVER_STANDARD);
  sim.setGravity({0, 0, -9.8});
  sim.setLightPosition(30, 0, 10);

  // add objects
  auto checkerboard = sim.addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setFrictionCoefficient(0.5);

  auto box = sim.addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5);
  box->setFrictionCoefficient(0.8);

  std::vector<ode_sim::SingleBodyHandle> objectList;

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim.addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5);
      ball->setFrictionCoefficient(1.0);
      objectList.push_back(ball);
    }
  }

  // camera relative position
  sim.cameraFollowObject(checkerboard, {10, 0, 5});

  // iteration counter
  int cnt = 0;

  // simulation loop
  // press 'q' key to quit
  while (sim.visualizerLoop(dt, 1)) {
    if (++cnt < forceMaxTime / dt)
      box->setExternalForce(force);

    sim.integrate(dt);
  }
  return 0;
}
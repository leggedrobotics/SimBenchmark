//
// Created by kangd on 11.02.18.
//

#include "BtSim.hpp"

enum Object {
  BOX,
  BALL,
  CAPSULE
};

int main() {

  bullet_sim::BtSim sim(800, 600, 0.5, bullet_sim::SOLVER_SEQUENTIAL_IMPULSE, benchmark::NO_BACKGROUND);
  sim.setGravity({0, 0, -9.8});
  sim.setLightPosition(30, 0, 10);

  // add objects
  auto checkerboard = sim.addCheckerboard(5.0, 10.0, 10.0, 0.1);

  Object object = BALL;
  switch(object) {
    case BOX:
    {
      auto box1 = sim.addBox(1, 1, 1, 1);
      box1->setPosition(0, 0, 0.6);
      auto box2 = sim.addBox(1, 1, 1, 1);
      box2->setPosition(0, 0, 1.7);
      auto box3 = sim.addBox(1, 1, 1, 1);
      box3->setPosition(0, 0, 2.8);
      auto box4 = sim.addBox(1, 1, 1, 1);
      box4->setPosition(0, 0, 3.9);
      break;
    }
    case BALL:
    {
      auto ball1 = sim.addSphere(0.5, 1);
      ball1->setPosition(0, 0, 0.5);
      auto ball2 = sim.addSphere(0.5, 1);
      ball2->setPosition(0, 0, 1.5);
      auto ball3 = sim.addSphere(0.5, 1);
      ball3->setPosition(0, 0, 2.5);
      auto ball4 = sim.addSphere(0.5, 1);
      ball4->setPosition(0, 0, 3.5);
      break;
    }
    case CAPSULE:
    {
      auto capsule1 = sim.addCapsule(0.25, 0.5, 1);
      capsule1->setPosition(-2.5, -2.5, 0.5);
      auto capsule2 = sim.addCapsule(0.25, 0.5, 1);
      capsule2->setPosition(-2.5, 2.5, 1.5);
      auto capsule3 = sim.addCapsule(0.25, 0.5, 1);
      capsule3->setPosition(2.5, -2.5, 2.5);
      auto capsule4 = sim.addCapsule(0.25, 0.5, 1);
      capsule4->setPosition(2.5, 2.5, 3.5);
      break;
    }
  }

  double dt = 0.01;  // (sec)

  // camera relative position
  sim.cameraFollowObject(checkerboard, {10, 0, 20});

  // simulation loop
  // press 'q' key to quit
  sim.loop(dt, 1.0);

  return 0;
}
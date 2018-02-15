//
// Created by kangd on 11.02.18.
//

#include "bulletSim/World_RG.hpp"

enum Object {
  BOX,
  BALL
};

int main() {

  bullet_sim::World_RG bulletSim(800, 600, 0.5,
                                 bullet_sim::NO_BACKGROUND,
                                 bullet_sim::SOLVER_SEQUENTIAL_IMPULSE);
  bulletSim.setGravity({0, 0, -9.8});
  bulletSim.setLightPosition(30, 0, 10);

  // add objects
  auto checkerboard = bulletSim.addCheckerboard(5.0, 10.0, 10.0, 0.1);

  Object object = BOX;
  switch(object) {
    case BOX:
    {
      auto box1 = bulletSim.addBox(1, 1, 1, 1);
      box1->setPosition(0, 0, 0.5);
      auto box2 = bulletSim.addBox(1, 1, 1, 1);
      box2->setPosition(5, 0, 1.5);
      auto box3 = bulletSim.addBox(1, 1, 1, 1);
      box3->setPosition(0, 5, 2.5);
      auto box4 = bulletSim.addBox(1, 1, 1, 1);
      box4->setPosition(5, 5, 3.5);
      break;
    }
    case BALL:
    {
      auto ball1 = bulletSim.addSphere(0.5, 1);
      ball1->setPosition(0, 0, 0.5);
      auto ball2 = bulletSim.addSphere(0.5, 1);
      ball2->setPosition(0, 0, 1.5);
      auto ball3 = bulletSim.addSphere(0.5, 1);
      ball3->setPosition(0, 0, 2.5);
      auto ball4 = bulletSim.addSphere(0.5, 1);
      ball4->setPosition(0, 0, 3.5);
      break;
    }
  }

  double dt = 0.01;  // (sec)

  // camera relative position
  bulletSim.cameraFollowObject(checkerboard, {10, 0, 5});

  // simulation loop
  // press 'q' key to quit
  bulletSim.loop(dt, 1.0);

  return 0;
}
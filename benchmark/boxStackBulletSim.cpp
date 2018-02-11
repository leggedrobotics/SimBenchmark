//
// Created by kangd on 11.02.18.
//

#include <iostream>
#include <btBulletDynamicsCommon.h>
#include "bulletSim/World_RG.hpp"

int main() {
  std::cout << "Hello, World!" << std::endl;

  const btScalar gravity = -9.81;

  bullet_sim::World_RG bulletSim(800, 600, 0.5);

  // objects
  auto box1 = bulletSim.addBox(1, 1, 1, 100);

  double dt = 0.01;  // (sec)

  // camera relative position
  bulletSim.cameraFollowObject(box1, {50, 10, 5});

  // simulation loop
  bulletSim.loop(dt, 1.0);

  return 0;
}
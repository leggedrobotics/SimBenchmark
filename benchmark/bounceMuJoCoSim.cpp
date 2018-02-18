//
// Created by kangd on 18.02.18.
//

#include "mujoco.h"
#include <iostream>
#include <mujocoSim/World.hpp>
#include "mujocoSim/SingleBodyObject.hpp"

char error[1000];

int main() {

  // load model from file and check for errors
  mujoco_sim::World world("/home/kangd/git/benchmark/mjpro150/model/hello.xml");

  // make objects
  mujoco_sim::object::SingleBodyObject ground(world.getWorldData(), 0);
  mujoco_sim::object::SingleBodyObject box1(world.getWorldData(), 1);
  mujoco_sim::object::SingleBodyObject box2(world.getWorldData(), 2);

  // run simulation for 10 seconds
  while( world.getWorldData()->time<10 ) {
    mj_step(world.getWorldModel(), world.getWorldData());
    RAIINFO(box1.getPosition());
  }

  return 0;
}

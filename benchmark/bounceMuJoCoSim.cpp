//
// Created by kangd on 18.02.18.
//

#include <mujoco.h>
#include <mujocoSim/World.hpp>

char error[1000];

int main() {

  // load model from file and check for errors
  mujoco_sim::World world("/home/kangd/git/benchmark/mjpro150/model/hello.xml");

  // run simulation for 10 seconds
  while( world.getWorldData()->time<10 ) {
    mj_step(world.getWorldModel(), world.getWorldData());
    RAIINFO(world.getObjectList()[1]->getPosition());
  }

  return 0;
}

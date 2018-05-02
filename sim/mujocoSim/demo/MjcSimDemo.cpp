//
// Created by kangd on 18.02.18.
//

#include "MjcWorld_RG.hpp"

int main() {
  // load model from file and check for errors
  mujoco_sim::MjcWorld_RG sim(800,
                           600,
                           0.5,
                           "../../../res/mujoco/test.xml",
                           "../mjkey.txt",
                           benchmark::NO_BACKGROUND,
                           mujoco_sim::SOLVER_NEWTON);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {10, 0, 5});

  // run simulation for 10 seconds
  sim.setTimeStep(0.01);
  sim.loop(1.0);

  return 0;
}

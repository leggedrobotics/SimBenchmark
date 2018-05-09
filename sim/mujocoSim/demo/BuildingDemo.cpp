//
// Created by kangd on 09.05.18.
//

#include <MjcWorld_RG.hpp>

int main() {

  // file path
  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/mujoco/Building/building.xml";

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../../../lib/mjpro150/mjkey.txt";

  // sim
  mujoco_sim::MjcWorld_RG sim(1280, 720, 0.1,
                              urdfPath.c_str(),
                              keyPath.c_str(),
                              benchmark::NO_BACKGROUND);

  RAIINFO("number of blocks "<< sim.getNumObject()-1);

  /// NOTE: dt = 0.01 is too large for realistic simulation
  const double dt = 0.001;  // (sec)

  // camera relative position
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {0, 5, 2});

  // simulation loop
  // press 'q' key to quit
  sim.setTimeStep(dt);
  sim.loop(1.0);

}

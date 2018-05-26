//
// Created by kangd on 26.05.18.
//

#include "BtMbSim.hpp"

int main(int argc, const char* argv[]) {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Singlebody/";

  bullet_multibody_sim::BtMbSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);

  auto anymal = sim.addArticulatedSystem(urdfPath, bullet_multibody_sim::object::URDF);
  sim.setGravity({0, 0, -9.8});
  sim.setTimeStep(0.005);

  while(sim.visualizerLoop(0.005, 1.0)) {
    sim.integrate();
  }
  return 0;
}
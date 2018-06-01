//
// Created by kangd on 24.05.18.
//

#include "MjcSim.hpp"

int main(int argc, const char* argv[]) {

  bool gui = true;
  if(argc == 2) {
    if(strcmp(argv[1], "--nogui") == 0)
      gui = false;
  }

  // file path
  std::string xmlPath(__FILE__);
  while (xmlPath.back() != '/')
    xmlPath.erase(xmlPath.size() - 1, 1);
  xmlPath += "../../../res/mujoco/Humanoid/humanoidA.xml";

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../../../lib/mjpro150/mjkey.txt";

  // load model from file and check for errors
  mujoco_sim::MjcSim sim(800,
                              600,
                              0.5,
                              xmlPath.c_str(),
                              keyPath.c_str(),
                              benchmark::NO_BACKGROUND,
                              mujoco_sim::SOLVER_NEWTON);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {2, 0, 2});
  sim.setTimeStep(0.03);

  StopWatch watch;
  watch.start();

  int numcontact = 0;
  int numiter = 100000;
  if(gui) {
    for(int i = 0; i < numiter && sim.visualizerLoop(0.03); i++) {
      sim.integrate();
      numcontact += sim.getWorldNumContacts();
    }
  } else {
    for(int i = 0; i < numiter; i++) {
      sim.integrate();
      numcontact += sim.getWorldNumContacts();
    }
  }

  double time =  watch.measure();
  RAIINFO("num contacts = " << double(numcontact/numiter));
  RAIINFO("100k takes " << time <<" seconds = " << 100.0 / time << " kHz");

  return 0;
}

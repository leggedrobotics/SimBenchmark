//
// Created by kangd on 19.11.17.
//
#include <raiSim/World_RG.hpp>

#define VISUALIZER_TEST

int main() {

  // timer
  std::string path = "/tmp";
  rai::Utils::timer->setLogPath(path);

  rai_sim::World_RG raiSim(800, 600, 0.5);

  raiSim.setGravity({0,0,0});
  raiSim.setLightPosition(30, 0, 10);

  // ball
  auto ball1 = raiSim.addSphere(2, 10.0);
  ball1.visual()[0]->setColor({1.0, 1.0, 0.0});
  ball1->setPosition(-20.0, 5.0, 0.0);
  ball1->setVelocity(20, 0, 0, 0, 0, 0);

  auto ball2 = raiSim.addSphere(2, 10.0);
  ball2.visual()[0]->setColor({1.0, 1.0, 0.0});
  ball2->setPosition(-20.0, -5.0, 0.0);
  ball2->setVelocity(20, 0, 0, 0, 0, 0);

  auto ball3 = raiSim.addSphere(2, 10.0);
  ball3.visual()[0]->setColor({1.0, 1.0, 0.0});
  ball3->setPosition(-20.0, 0.0, 5.0);
  ball3->setVelocity(20, 0, 0, 0, 0, 0);

  auto ball4 = raiSim.addSphere(2, 10.0);
  ball4.visual()[0]->setColor({1.0, 1.0, 0.0});
  ball4->setPosition(-20.0, 0.0, -5.0);
  ball4->setVelocity(20, 0, 0, 0, 0, 0);

  // build wall
  float shortLen = 1;
  float longLen = 2;
  float heightLen = 1;

  std::vector<rai_sim::SingleBodyHandle> objectPtrList;
  for(int i = 0; i < 10; i++) {
    for(int j = 0; j < 20; j++) {
      auto block = raiSim.addBox(shortLen, longLen, heightLen, 1.0);

      block.visual()[0]->setColor({1.0, 0, 0});
      block->setPosition(0, longLen * i - longLen * 5, heightLen * j - heightLen * 10);

      objectPtrList.push_back(block);
    }
  }

  /// NOTE: dt = 0.01 is too large for realistic simulation
  double dt = 0.01;  // (sec)

  // camera relative position
  raiSim.cameraFollowObject(ball1, {50, 10, 5});

  // simulation loop
  // press 'q' key to quit
  raiSim.loop(dt, 1.0);

  RAIINFO("demoTower ended");
}

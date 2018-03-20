//
// Created by kangd on 20.03.18.
//

#include <dartSim/World_RG.hpp>

int main() {

  dart_sim::World_RG dartSim(800, 600, 0.5);

  double dt = 0.01;  // (sec)
  dartSim.setGravity({0,0,0});
  dartSim.setLightPosition(30, 0, 10);
  dartSim.setTimeStep(dt);

  // ball
  auto ball1 = dartSim.addSphere(2, 10.0);
//  ball1.visual()[0]->setColor({1.0, 1.0, 0.0});
  ball1->setPosition(-20.0, 5.0, 0.0);
  ball1->setVelocity(20, 0, 0, 0, 0, 0);

  auto ball2 = dartSim.addSphere(2, 10.0);
//  ball2.visual()[0]->setColor({1.0, 1.0, 0.0});
  ball2->setPosition(-20.0, -5.0, 0.0);
  ball2->setVelocity(20, 0, 0, 0, 0, 0);

  auto ball3 = dartSim.addSphere(2, 10.0);
//  ball3.visual()[0]->setColor({1.0, 1.0, 0.0});
  ball3->setPosition(-20.0, 0.0, 5.0);
  ball3->setVelocity(20, 0, 0, 0, 0, 0);

  auto ball4 = dartSim.addSphere(2, 10.0);
//  ball4.visual()[0]->setColor({1.0, 1.0, 0.0});
  ball4->setPosition(-20.0, 0.0, -5.0);
  ball4->setVelocity(20, 0, 0, 0, 0, 0);

  // build wall
  float shortLen = 1;
  float longLen = 2;
  float heightLen = 1;

  std::vector<benchmark::SingleBodyHandle> objectPtrList;
  for(int i = 0; i < 10; i++) {
    for(int j = 0; j < 20; j++) {
      auto block = dartSim.addBox(shortLen, longLen, heightLen, 1.0);

//      block.visual()[0]->setColor({1.0, 0, 0});
      block->setPosition(0, longLen * i - longLen * 5, heightLen * j - heightLen * 10);

      objectPtrList.push_back(block);
    }
  }

  // camera relative position
  dartSim.cameraFollowObject(ball1, {50, 10, 5});

  // simulation loop
  // press 'q' key to quit
  dartSim.loop(dt, 1.0);
}

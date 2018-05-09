//
// Created by kangd on 09.05.18.
//

#include <OdeWorld_RG.hpp>

int main() {

  // sim
  ode_sim::OdeWorld_RG sim(1280, 720, 0.1, benchmark::NO_BACKGROUND, ode_sim::SOLVER_QUICK);
  sim.setERP(0.2, 0, 0);

  // object
  auto checkerboard = sim.addCheckerboard(10.0, 400.0, 400.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

  // build tower
  const float shortLen = 0.05;
  const float longLen = 0.2;
  const float heightLen = 0.1;

  // num of blocks = numFloor x numBase + numFloor x (numWall x 2 + 1)
  const int numFloor = 6;
  const int numBase = 20;
  const int numWall = numBase / 2;

  std::vector<benchmark::SingleBodyHandle> objectPtrList;
  for(int i = 0; i < numFloor; i++) {
    // i floor
    for(int j = 0; j < numBase; j++) {
      // base
      auto base = sim.addBox(shortLen, longLen + 0.05, heightLen, 10.0);
      base.visual()[0]->setColor({0.0, 0.0, 1.0});
      base->setPosition(j * longLen, 0, i * heightLen * 2 + 0.05);
      objectPtrList.push_back(base);
    }

    for(int j = 0; j < numWall; j++) {
      // right wall
      auto wall = sim.addBox(longLen, shortLen, heightLen, 10.0);
      wall.visual()[0]->setColor({0.0, 1.0, 0.0});
      wall->setPosition(j * longLen * 2 + 0.1, -0.5 * longLen, i * heightLen * 2 + 0.15);
      objectPtrList.push_back(wall);
    }

    for(int j = 0; j < numWall - 1; j++) {
      // left wall
      auto wall = sim.addBox(longLen, shortLen, heightLen, 10.0);
      wall.visual()[0]->setColor({1.0, 0.0, 0.0});
      wall->setPosition(j * longLen * 2 + 0.3, 0.5 * longLen, i * heightLen * 2 + 0.15);
      objectPtrList.push_back(wall);
    }

    // first wall on left
    auto wall1 = sim.addBox(longLen, shortLen, heightLen, 10.0);
    wall1.visual()[0]->setColor({1.0, 0.0, 0.0});
    wall1->setPosition(0.1, 0.5 * longLen, i * heightLen * 2 + 0.15);
    objectPtrList.push_back(wall1);

    // last wall on left
    auto wall2 = sim.addBox(longLen, shortLen, heightLen, 10.0);
    wall2.visual()[0]->setColor({1.0, 0.0, 0.0});
    wall2->setPosition((numWall - 1) * longLen * 2 + 0.1, 0.5 * longLen, i * heightLen * 2 + 0.15);
    objectPtrList.push_back(wall2);
  }

  RAIINFO("number of blocks "<< objectPtrList.size());

  /// NOTE: dt = 0.01 is too large for realistic simulation
  const double dt = 0.001;  // (sec)

  // camera relative position
  sim.cameraFollowObject(checkerboard, {0, 5, 2});

  // simulation loop
  // press 'q' key to quit
  sim.loop(dt, 1);

}

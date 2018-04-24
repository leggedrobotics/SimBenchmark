//
// Created by kangd on 23.04.18.
//

#include <OdeWorld_RG.hpp>

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Multibody/";

  ode_sim::OdeWorld_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  sim.setGravity({0, 0, 0});

  /// NOTE erp should be set to 0.2 for articulated system simulation on ODE
  sim.setERP(0.2, 0.2, 0.2);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);
  auto robot = sim.addArticulatedSystem(urdfPath);
  robot->setGeneralizedCoordinate(
      {0, 0, 1,
       1, 0, 0, 0,
       0, 0, 0});

  sim.cameraFollowObject(checkerboard, {0, 2.5, 1});
  for(int i = 0; i < 10000 && sim.visualizerLoop(0.005, 0.1); i++) {
//    robot->setGeneralizedForce(
//        {0, 0, 0,
//         0, 0, 0,
//         0, 0, 0.1});
    sim.integrate(0.005);
  }

  RAIINFO("generalized coordinate = " << robot->getGeneralizedCoordinate());

  return 0;
}

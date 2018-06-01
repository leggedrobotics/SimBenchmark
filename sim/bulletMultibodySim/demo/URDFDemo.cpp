//
// Created by kangd on 26.05.18.
//

#include "BtMbSim.hpp"

int main(int argc, const char* argv[]) {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Multibody/robot.urdf";

  bullet_mb_sim::BtMbSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  sim.setGravity({0, 0, -9.81});
  sim.setERP(0.2, 0.2, 0.2);

  double dt = 0.005;
  sim.setTimeStep(dt);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  sim.cameraFollowObject(checkerboard, {5, 0, 5});

  auto robot = sim.addArticulatedSystem(urdfPath, bullet_mb_sim::object::URDF);
  robot->setGeneralizedCoordinate({0, 0, 0.51,
                                   1, 0, 0, 0,
                                   0});

  int cnt = 0;
  while(sim.visualizerLoop(dt, 1.0)) {
    sim.integrate();
    if(cnt++ < 4000) {
      robot->setGeneralizedForce({0, 0, 0,
                                  0, 0, 0,
                                  0.1});
    } else {
      break;
    }
  }

  RAIINFO(std::endl << "genvel = " << std::endl << robot->getGeneralizedVelocity());
  RAIINFO(std::endl << "momentum = " << std::endl << robot->getLinearMomentumInCartesianSpace());
  RAIINFO(std::endl << "energy = " << std::endl << robot->getEnergy({0, 0, -9.81}));
  return 0;
}
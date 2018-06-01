//
// Created by jhwangbo on 04/12/17.
//
#include <raiSim/World_RG.hpp>
#include "raiSim/StopWatch.hpp"

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Atlas/robot.urdf";

  rai_sim::World_RG sim(1280, 720, 0.2, rai_sim::NO_BACKGROUND);
  sim.setLightPosition(0, -10, 10);

  auto atlas = sim.addArticulatedSystem(urdfPath);
  auto checkerBoard = sim.addCheckerboard(2, 100, 100, 0.1, 1, -1, rai_sim::GRID);

  Eigen::VectorXd gc(atlas->getStateDim());
  Eigen::VectorXd gv(atlas->getDOF()), tau(atlas->getDOF());

  std::cout<<"dof "<<atlas->getDOF()<<"\n";

  atlas->printOutBodyNamesInOrder();

  gc.setZero();
  gc.segment<7>(0) << 0,0,3,1,0,0,0;
  gv.setZero();
  tau.setZero();

  atlas->setState(gc, gv);

  const double dt = 1.0 / 5000.0;
  int counter = 0;
  sim.cameraFollowObject(atlas, {5, 0, 0});

  sim.loop(dt);

  std::cout<<"contact problem size: "<<sim.getContactProblem().size()<<"\n";

//  StopWatch watch;
//  watch.start();
//  for(int i=0; i<10000; i++)
//    sim.integrate(dt);
//  std::cout<<"Number of steps in one second  "<<10.0/watch.measure()<<"k\n";
}
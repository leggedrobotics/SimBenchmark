//
// Created by jhwangbo on 04/12/17.
//
#include <raiSim/World_RG.hpp>
#include "raiSim/StopWatch.hpp"

#define GUI

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/demo/ode-rai-dart/Atlas/robot.urdf";

#ifdef GUI
  rai_sim::World_RG sim(800, 600, 0.2, rai_sim::NO_BACKGROUND);
#else
  rai_sim::World_RG sim;
#endif

  auto atlas = sim.addArticulatedSystem(urdfPath);
  auto checkerBoard = sim.addCheckerboard(2, 100, 100, 0.1, -1, rai_sim::GRID);

  Eigen::VectorXd gc(atlas->getGeneralizedCoordinateDim());
  Eigen::VectorXd gv(atlas->getDOF()), tau(atlas->getDOF());

  std::cout<<"dof "<<atlas->getDOF()<<"\n";

  atlas->printOutBodyNamesInOrder();

  gc.setZero();
//  gc.segment<7>(0) << 0,0,3,1,0,0,0;
  gc.segment<7>(0) << 0,0,0.3,0.7071,0,0.7071,0;
  gv.setZero();
  tau.setZero();

  atlas->setState(gc, gv);

  const double dt = 1.0 / 5000.0;
  int counter = 0;
  sim.setTimeStep(dt);
//  sim.loop();

  StopWatch watch;
  watch.start();

#ifdef GUI
  sim.setLightPosition(0, -10, 10);
  sim.cameraFollowObject(atlas, {5, 0, 0});
  for(int i=0; i<100000 && sim.visualizerLoop(); i++)
#else
    for(int i=0; i<100000; i++)
#endif
  {
    sim.integrate();
    counter += sim.getContactProblem().size();
  }
  std::cout<<"Number of steps in one second  "<<100.0/watch.measure()<<"k\n";
  std::cout<<"contact problem size: "<< float(counter)/100000 <<"\n";
}
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
  sim.setERP(0);

  auto atlas = sim.addArticulatedSystem(urdfPath);
  auto checkerBoard = sim.addCheckerboard(2, 100, 100, 0.1, -1, rai_sim::GRID);

  // states
  Eigen::VectorXd gc(atlas->getGeneralizedCoordinateDim());
  Eigen::VectorXd gv(atlas->getDOF());
  Eigen::VectorXd tau(atlas->getDOF());

  // gains
  Eigen::VectorXd kp(atlas->getDOF()), kd(atlas->getDOF());
  tau.setZero(); kp.setZero(); kd.setZero();
  kp<< 0,0,0,0,0,0,
      10000,10000,10000,
      20000,10000,1000,1000,1000, 100,.2,
      20000,10000,1000,1000,1000, 100,.2,
      20000,5000,5000,300,100,20000,
      20000,5000,5000,300,100,20000;
  kd<< 0,0,0,0,0,0,
      50,   50,   50,
      30,   10,   10,  5,   3,    0.5,.01,
      30,   10,   10,  5,   3,    0.5,.01,
      40,   40,  40,  5,  2,  10,
      40,   40,  40,  5,  2,  10;

  std::cout<<"dof "<<atlas->getDOF()<<"\n";
  atlas->printOutBodyNamesInOrder();

  // initial state
  gc.setZero();
  gc.segment<7>(0) << 0,0,1,1,0,0,0;
  gv.setZero();
  tau.setZero();

  atlas->setState(gc, gv);

  const double dt = 1.0 / 5000.0;
  int counter = 0;
  sim.setTimeStep(dt);

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
    sim.integrate1();
    gc = atlas->getGeneralizedCoordinate();
    gv = atlas->getGeneralizedVelocity();
    tau.tail(30) = -kp.tail(30).cwiseProduct(gc.tail(30)) - kd.cwiseProduct(gv).tail(30);
    atlas->setGeneralizedForce(tau);
    sim.integrate2();
    counter += sim.getContactProblem().size();
  }
  std::cout<<"Number of steps in one second  "<<100.0/watch.measure()<<"k\n";
  std::cout<<"contact problem size: "<< float(counter)/100000 <<"\n";
}
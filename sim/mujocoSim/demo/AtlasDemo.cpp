//
// Created by jhwangbo on 04/12/17.
//
#include <MjcSim.hpp>
#include <string>
#include "raiCommon/utils/StopWatch.hpp"

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Atlas/robot.urdf";

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../../../lib/mjpro150/mjkey.txt";

  mujoco_sim::MjcSim sim(800, 600, 0.2,
                              urdfPath.c_str(),
                              keyPath.c_str(),
                              benchmark::NO_BACKGROUND);
  sim.setLightPosition(0, -10, 10);

  Eigen::VectorXd gc(sim.getStateDimension());
  Eigen::VectorXd gv(sim.getDOF()), tau(sim.getDOF());

  std::cout<<"dof "<<sim.getDOF()<<"\n";

  gc.setZero();
  gc.segment<7>(0) << 0,0,3,1,0,0,0;
  gv.setZero();
  tau.setZero();

  sim.setState(gc, gv);

  const double dt = 1.0 / 5000.0;
  int counter = 0;
//  sim.cameraFollowObject(, {5, 0, 0});
  sim.setTimeStep(dt);

  StopWatch watch;
  watch.start();
  for(int i=0; i<10000 && sim.visualizerLoop(dt); i++)
    sim.integrate();

  std::cout<<"Number of steps in one second  "<<10.0/watch.measure()<<"k\n";
  std::cout<<"contact problem size: "<< sim.getWorldNumContacts() <<"\n";
}
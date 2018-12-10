//
// Created by kangd on 18.02.18.
//

#include <boost/program_options.hpp>

#include "MjcSim.hpp"
#include "raiCommon/utils/StopWatch.hpp"

//#define VIDEO_SAVE_MODE

namespace po = boost::program_options;
std::vector<double> kenergy;

void showplot(double);

int main(int argc, const char* argv[]) {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/mujoco/ANYmal/robot-energy.urdf";

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../../../lib/mjpro150/mjkey.txt";

  mujoco_sim::MjcSim
      sim(800, 600, 0.5, urdfPath.c_str(), keyPath.c_str(), benchmark::NO_BACKGROUND);
  sim.cameraFollowObject(sim.getSingleBodyHandle(sim.getNumObject()-1), {1.0, 1.0, 1.0});

  /// no slip parameter should be set in order to make anymal stands
  sim.setTimeStep(0.005);

  // set color
  for(int i = 0; i < sim.getNumObject()-1; i++) {
    sim.getSingleBodyHandle(i).visual()[0]->setColor({0, 0, 1});
  }

  // initial general coordinate
  sim.setGeneralizedCoordinate(
      {0, 0, 10,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8,
       -0.03, 0.4, -0.8,
       0.03, -0.4, 0.8,
       -0.03, -0.4, 0.8});
  sim.setGeneralizedVelocity(
      {0, 0, 0,
       0.2, 1.0, 0.5,
       0.0, 0.0, 0.0,
       0.0, 0.0, 0.0,
       0.0, 0.0, 0.0,
       0.0, 0.0, 0.0});

  for(int i = 0 ; i < sim.getNumObject(); i++) {
    static_cast<mujoco_sim::object::MjcSingleBodyObject *>(sim.getSingleBodyHandle(i).s_)
        ->setCollisionGroupAndMask(1, 0);
  }

  double g = -0;
  double dt = 0.005;
  sim.setGravity({0, 0, g});
  sim.setTimeStep(dt);

  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {2.0, 0.0, 0.5});

  double E0 = 0;
  for(int i = 0; i < int(5.0/dt) && sim.visualizerLoop(dt, 1.0); i++) {
    sim.integrate1();
    if(i == 0) E0 = sim.getEnergy({0, 0, g});
    kenergy.push_back(sim.getEnergy({0, 0, g}));
    sim.integrate2();
  }

  RAIINFO("initial E = " << E0)
  showplot(E0);
  return 0;
}

void showplot(double E0) {

  int n = kenergy.size();
  Eigen::MatrixXd tdata(n, 1);
  Eigen::MatrixXd Edata(n, 1);
  Eigen::MatrixXd edata(n, 1);

  for(int i = 0; i < n; i++) {
    tdata(i, 0) = i;
    Edata(i, 0) = kenergy[i];
    edata(i, 0) = E0 - kenergy[i];
  }

  rai::Utils::Graph::FigProp2D figure1properties("step", "Energy", "Energy");
  rai::Utils::graph->figure(1, figure1properties);
  rai::Utils::graph->appendData(1,
                                tdata.data(),
                                Edata.data(),
                                n,
                                "E");
  rai::Utils::graph->drawFigure(1);

  rai::Utils::Graph::FigProp2D figure2properties("step", "Energy error", "Energy error");
  rai::Utils::graph->figure(2, figure2properties);
  rai::Utils::graph->appendData(2,
                                tdata.data(),
                                edata.data(),
                                n,
                                "error");
  rai::Utils::graph->drawFigure(2);
  rai::Utils::graph->waitForEnter();
}

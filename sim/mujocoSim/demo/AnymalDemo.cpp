//
// Created by kangd on 18.02.18.
//

#include <boost/program_options.hpp>

#include "MjcWorld_RG.hpp"
#include "raiCommon/utils/StopWatch.hpp"

//#define VIDEO_SAVE_MODE

namespace po = boost::program_options;

int main(int argc, const char* argv[]) {

  // file path
  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/mujoco/ANYmal/robot2.urdf";

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../../../lib/mjpro150/mjkey.txt";

  mujoco_sim::MjcWorld_RG sim(800,
                              600,
                              0.5,
                              urdfPath.c_str(),
                              keyPath.c_str(),
                              benchmark::NO_BACKGROUND);
  sim.cameraFollowObject(sim.getSingleBodyHandle(sim.getNumObject()-1), {1.0, 1.0, 1.0});

  // set color
  for(int i = 0; i < sim.getNumObject()-1; i++) {
    sim.getSingleBodyHandle(i).visual()[0]->setColor({0, 0, 1});
  }

  // initial general coordinate
//  sim.setGeneralizedCoordinate(
//      {0, 0, 0.54,
//       1.0, 0.0, 0.0, 0.0,
//       0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
//  sim.setGeneralizedVelocity(Eigen::VectorXd::Zero(sim.getDOF()));
//  sim.setGeneralizedForce(Eigen::VectorXd::Zero(sim.getDOF()));

  // run simulation for 10 seconds
  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);

  const double kp = 400.0, kd = 1.0;

  jointNominalConfig << 0, 0, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

#if defined(VIDEO_SAVE_MODE)
  sim.startRecordingVideo("/tmp", "mjcAnymal");
  for(int i = 0; i < 2000 && sim.visualizerLoop(0.005, 1.0); i++) {
#endif
  while(sim.visualizerLoop(0.005, 1.0)) {
//    jointState = sim.getGeneralizedCoordinate();
//    jointVel = sim.getGeneralizedVelocity();
//    jointForce = sim.getGeneralizedForce();

//    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
//    jointForce.head(6).setZero();
//    sim.setGeneralizedForce(jointForce);
    sim.integrate(0.005);
  }

#if defined(VIDEO_SAVE_MODE)
  sim.stopRecordingVideo();
#endif
  return 0;
}

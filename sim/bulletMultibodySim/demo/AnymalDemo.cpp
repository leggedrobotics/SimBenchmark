//
// Created by kangd on 18.02.18.
//

#include <boost/program_options.hpp>

#include "BtMbSim.hpp"
#include "raiCommon/utils/StopWatch.hpp"

//#define VIDEO_SAVE_MODE

namespace po = boost::program_options;

int main(int argc, const char* argv[]) {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/ANYmal/robot.urdf";

  bullet_mb_sim::BtMbSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0.8);

  auto anymal = sim.addArticulatedSystem(urdfPath, bullet_mb_sim::object::URDF);
  anymal->setGeneralizedCoordinate(
      {0, 0, 0.54,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8,
       -0.03, 0.4, -0.8,
       0.03, -0.4, 0.8,
       -0.03, -0.4, 0.8});
//  anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
//  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  sim.setGravity({0, 0, -9.81});
  sim.setERP(0.05, 0.01, 0.0);
  sim.setTimeStep(0.005);

  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = 40.0, kd = 1.0;

  jointNominalConfig << 0, 0, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

  sim.cameraFollowObject(checkerboard, {1.0, 1.0, 1.0});

#if defined(VIDEO_SAVE_MODE)
  sim.startRecordingVideo("/tmp", "btAnymal");
  for(int i = 0; i < 2000 && sim.visualizerLoop(0.005, 1.0); i++) {
#else
  while(sim.visualizerLoop(0.005, 1.0)) {
#endif
//    jointState = anymal->getGeneralizedCoordinate();
//    jointVel = anymal->getGeneralizedVelocity();
//    jointForce = anymal->getGeneralizedForce();

//    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
//    jointForce.head(6).setZero();
//    anymal->setGeneralizedForce(jointForce);
    sim.integrate();
  }

#if defined(VIDEO_SAVE_MODE)
  sim.stopRecordingVideo();
#endif

  return 0;
}

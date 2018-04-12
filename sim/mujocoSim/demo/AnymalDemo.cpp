//
// Created by kangd on 18.02.18.
//

#include <boost/program_options.hpp>

#include "World_RG.hpp"
#include "raiCommon/utils/StopWatch.hpp"

#define SIM_TIME_MODE
#define VIDEO_SAVE_MODE

namespace po = boost::program_options;

int main(int argc, const char* argv[]) {

  // argument input
  po::options_description desc("generic");
  desc.add_options()
      ("solver", po::value<std::string>(), "contact solver (pgs / cg / newton)")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  mujoco_sim::SolverOption solverOption = mujoco_sim::SOLVER_PGS;
  if(vm.count("solver")) {
    if(vm["solver"].as<std::string>().compare("pgs") == 0) {
      solverOption = mujoco_sim::SOLVER_PGS;
    }
    else if(vm["solver"].as<std::string>().compare("cg") == 0) {
      solverOption = mujoco_sim::SOLVER_CG;
    }
    else if(vm["solver"].as<std::string>().compare("newton") == 0) {
      solverOption = mujoco_sim::SOLVER_NEWTON;
    } else {
      RAIFATAL("invalid solver option")
    }
  }

  // file path
  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/mujoco/ANYmal/robot.urdf";

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../../../lib/mjpro150/mjkey.txt";

#if defined(SIM_TIME_MODE)
  mujoco_sim::World_RG sim(urdfPath.c_str(),
                           keyPath.c_str(),
                           solverOption);
#else
  mujoco_sim::World_RG sim(800,
                           600,
                           0.5,
                           urdfPath.c_str(),
                           keyPath.c_str(),
                           benchmark::NO_BACKGROUND,
                           solverOption);
  sim.cameraFollowObject(sim.getSingleBodyHandle(sim.getNumObject()-1), {1.0, 1.0, 1.0});

  // set color
  for(int i = 0; i < sim.getNumObject()-1; i++) {
    sim.getSingleBodyHandle(i).visual()[0]->setColor({0, 0, 1});
  }
#endif

  // initial general coordinate
  sim.setGeneralizedCoordinate(
      {0, 0, 0.54,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
  sim.setGeneralizedVelocity(Eigen::VectorXd::Zero(sim.getDOF()));
  sim.setGeneralizedForce(Eigen::VectorXd::Zero(sim.getDOF()));

  // run simulation for 10 seconds
  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);

  // TODO why kp should be 400 for mjc?
  const double kp = 400.0, kd = 1.0;

  jointNominalConfig << 0, 0, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

#if defined(SIM_TIME_MODE)
  StopWatch watch;
  watch.start();
  for(int i = 0; i < 50000; i++) {
#else
#if defined(VIDEO_SAVE_MODE)
  sim.startRecordingVideo("/tmp", "mjcAnymal");
#endif
  for(int i = 0; i < 2000 && sim.visualizerLoop(0.005, 1.0); i++) {
#endif
    jointState = sim.getGeneralizedCoordinate();
    jointVel = sim.getGeneralizedVelocity();
    jointForce = sim.getGeneralizedForce();

    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
    jointForce.head(6).setZero();
    sim.setGeneralizedForce(jointForce);
    sim.integrate(0.005);
  }

#if defined(SIM_TIME_MODE)
  std::cout<<"time taken for 50k steps "<<watch.measure()<<"s \n";
#elif defined(VIDEO_SAVE_MODE)
  sim.stopRecordingVideo();
#endif
  return 0;
}

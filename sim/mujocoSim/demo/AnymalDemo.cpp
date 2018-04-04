//
// Created by kangd on 18.02.18.
//

#include "World_RG.hpp"
#include "raiCommon/utils/StopWatch.hpp"

#define SIM_TIME_MODE

int main() {

#ifdef SIM_TIME_MODE
  mujoco_sim::World_RG sim("../../../res/mujoco/ANYmal/robot.urdf",
                           "../mjkey.txt",
                           mujoco_sim::SOLVER_NEWTON);
#else
  mujoco_sim::World_RG sim(800,
                           600,
                           0.5,
                           "../../../res/mujoco/ANYmal/robot.urdf",
                           "../mjkey.txt",
                           benchmark::NO_BACKGROUND,
                           mujoco_sim::SOLVER_NEWTON);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {4, 0, 2});
#endif

  sim.setGeneralizedCoordinate(
      {0, 0, 0.6,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
  sim.setGeneralizedVelocity(Eigen::VectorXd::Zero(sim.getDOF()));
  sim.setGeneralizedForce(Eigen::VectorXd::Zero(sim.getDOF()));

  // run simulation for 10 seconds
  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);

  // TODO check why kp should be 400?
  const double kp = 400.0, kd = 1.0;

  jointNominalConfig << 0, 0, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

#ifdef SIM_TIME_MODE
  StopWatch watch;
  watch.start();
  for(int i = 0; i < 10000; i++) {
#else
  while(sim.visualizerLoop(0.005, 1.0)) {
#endif
    jointState = sim.getGeneralizedCoordinate();
    jointVel = sim.getGeneralizedVelocity();
    jointForce = sim.getGeneralizedForce();

    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
    jointForce.head(6).setZero();
    sim.setGeneralizedForce(jointForce);
    sim.integrate(0.005);
  }

#ifdef SIM_TIME_MODE
  std::cout<<"time taken for 10k steps "<<watch.measure()<<"s \n";
#endif

  return 0;
}

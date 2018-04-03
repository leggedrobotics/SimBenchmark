//
// Created by kangd on 18.02.18.
//

#include "World_RG.hpp"

int main() {
  // load model from file and check for errors
  mujoco_sim::World_RG sim(800,
                           600,
                           0.5,
                           "../../../res/mujoco/ANYmal/robot.urdf",
                           "../mjkey.txt",
                           benchmark::NO_BACKGROUND,
                           mujoco_sim::SOLVER_NEWTON);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {10, 0, 5});

  sim.setGeneralizedCoordinate(
      {0, 0, 0.6,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
  sim.setGeneralizedVelocity(Eigen::VectorXd::Zero(sim.getDOF()));
  sim.setGeneralizedForce(Eigen::VectorXd::Zero(sim.getDOF()));

  // run simulation for 10 seconds
  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = 40.0, kd = 1.0;

  jointNominalConfig << 0, 0, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

  while(sim.visualizerLoop(0.005, 1.0)) {
    jointState = sim.getGeneralizedCoordinate();
    jointVel = sim.getGeneralizedVelocity();
    jointForce = sim.getGeneralizedForce();

    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
    jointForce.head(6).setZero();
    sim.setGeneralizedForce(jointForce);
    sim.integrate(0.005);
  }

  return 0;
}

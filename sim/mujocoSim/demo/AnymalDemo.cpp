//
// Created by kangd on 18.02.18.
//

#include "World_RG.hpp"

int main() {
  // load model from file and check for errors
  mujoco_sim::World_RG sim(800,
                           600,
                           0.5,
                           "../../../res/ANYmal/robot.urdf",
                           "../mjkey.txt",
                           benchmark::NO_BACKGROUND,
                           mujoco_sim::SOLVER_NEWTON);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {10, 0, 5});

  // run simulation for 10 seconds
  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = 40.0, kd = 1.0;

  jointNominalConfig << 5, 5, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

  RAIINFO(sim.getDOF());
  RAIINFO(sim.getGeneralizedCoordinateDim());

  sim.loop(0.01, 1.0);
  return 0;
}

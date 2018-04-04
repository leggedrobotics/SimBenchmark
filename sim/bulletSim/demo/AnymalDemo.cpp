/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <World_RG.hpp>
#include "raiCommon/utils/StopWatch.hpp"

#define SIM_TIME_MODE

int main() {

#ifdef SIM_TIME_MODE
  bullet_sim::World_RG sim(bullet_sim::SOLVER_MULTI_BODY);
#else
  bullet_sim::World_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND, bullet_sim::SOLVER_MULTI_BODY);
#endif

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, 1, -1);
  auto anymal = sim.addArticulatedSystem("../../../res/ANYmal/robot.urdf");
  anymal->setGeneralizedCoordinate(
      {5, 5, 0.6,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
  anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  sim.setGravity({0, 0, -9.8});

  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = 40.0, kd = 1.0;

  jointNominalConfig << 5, 5, 0,
      1.0, 0, 0, 0,
      0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

#ifdef SIM_TIME_MODE
  StopWatch watch;
  watch.start();
  for(int i = 0; i < 10000; i++) {
#else
    sim.cameraFollowObject(checkerboard, {10, 10, 15});
  while(sim.visualizerLoop(0.005, 1.0)) {
#endif
    jointState = anymal->getGeneralizedCoordinate();
    jointVel = anymal->getGeneralizedVelocity();
    jointForce = anymal->getGeneralizedForce();

    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
    jointForce.head(6).setZero();
    anymal->setGeneralizedForce(jointForce);
    sim.integrate(0.005);
  }

#ifdef SIM_TIME_MODE
  std::cout<<"time taken for 10k steps "<< watch.measure()<<"s \n";
#endif

  return 0;
}
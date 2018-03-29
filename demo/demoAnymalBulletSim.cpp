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

#include <bulletSim/World_RG.hpp>

int main() {

  bullet_sim::World_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND, bullet_sim::SOLVER_MULTI_BODY);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, 1, -1, benchmark::GRID);
  auto anymal = sim.addArticulatedSystem("../res/ANYmal/robot.urdf");
  anymal->setGeneralizedCoordinate(
      {0, 0, 0.54,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});

  sim.setGravity({0, 0, -9.8});
  sim.cameraFollowObject(checkerboard, {10, 10, 15});
  sim.loop(0.01);

  return 0;
}
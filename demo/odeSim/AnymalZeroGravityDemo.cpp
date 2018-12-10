//
// Created by kangd on 15.04.18.
//

#include <OdeSim.hpp>
#include "raiCommon/utils/StopWatch.hpp"

#define URDFPATH ROOTPATH "/res/demo/ode-rai-dart/ANYmal-nomesh/robot.urdf"
//#define VIDEO_SAVE_MODE

int main() {

    std::string urdfPath = std::string(URDFPATH);

    ode_sim::OdeSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);
    sim.setGravity({0, 0, 0});

    auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);
    checkerboard->setFrictionCoefficient(0.8);

    auto anymal = sim.addArticulatedSystem(urdfPath, 1, 0);
    anymal->setGeneralizedCoordinate(
            {0, 0, 2,
             1.0, 0.0, 0.0, 0.0,
             0.03, 0.4, -0.8,
             -0.03, 0.4, -0.8,
             0.03, -0.4, 0.8,
             -0.03, -0.4, 0.8});
    anymal->setGeneralizedVelocity(
            {0, 0, 0,
             0, 0, 0,
             0, 0, 1,
             0, 0, 0,
             0, 0, 0,
             0, 0, 0});

    sim.cameraFollowObject(checkerboard, {5.0, 0.0, 5.0});
    for(int i = 0; i > -1 && sim.visualizerLoop(0.005, 1.0); i++) {
        sim.integrate(0.005);
    }

    RAIINFO(anymal->getGeneralizedVelocity())
    return 0;
}


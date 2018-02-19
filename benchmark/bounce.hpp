//
// Created by kangd on 17.02.18.
//

#ifndef BENCHMARK_BOUNCE_HPP
#define BENCHMARK_BOUNCE_HPP

#include <Eigen/Geometry>

namespace benchmark {

// path
std::string parentDir = "/home/kangd/Desktop/raisim-benchmark/log/bounce/";

// sim properties
double lightX = 30.0;
double lightY = 0.0;
double lightZ = 10.0;

// parameters
double dt = 0.01;                     // time step
const int simulationTime = 10;        // time for simulation force
double restitution = 1.0;             // restitution coefficient
double friction = 0.0;                // friction coefficient
double dropHeight = 10.0;

Eigen::Vector3d gravity = {0, 0, -9.8};

// object
double ballR = 0.5;
double ballM = 1.0;

}

#endif //BENCHMARK_BOUNCE_HPP

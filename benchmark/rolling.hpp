//
// Created by kangd on 17.02.18.
//

#ifndef BENCHMARK_ROLLING_HPP
#define BENCHMARK_ROLLING_HPP

#include "benchmark.hpp"

namespace benchmark {

// path
std::string parentDir = "rolling/";

// option
enum ForceDirection { FORCE_Y, FORCE_XY };

bool visualize = false;
ForceDirection forceDirection = FORCE_XY;

// sim properties
double lightX = 30.0;
double lightY = 0.0;
double lightZ = 10.0;

// parameters
double dt = 0.01;                          // time step
const int simulationTime = 4;               // time for applying force

Eigen::Vector3d forceY = {0, 150, 0};           // force along Y
Eigen::Vector3d forceXY = {106.0660171779821, 106.0660171779821, 0};        // force along XY
Eigen::Vector3d gravity = {0, 0, -9.8};

double erp = 0.0;                           // bullet and ode

double groundMu = 0.5;
double ballMu = 1.0;
double boxMu = 0.8;

double initPenetration = 1e-6;

}

#endif //BENCHMARK_ROLLING_HPP

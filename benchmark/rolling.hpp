//
// Created by kangd on 17.02.18.
//

#ifndef BENCHMARK_ROLLING_HPP
#define BENCHMARK_ROLLING_HPP

namespace benchmark {

// sim properties
double lightX = 30.0;
double lightY = 0.0;
double lightZ = 10.0;

// parameters
double dt = 0.001;                      // time step
const int simulationTime = 3;             // time for applying force
Eigen::Vector3d force = {0, 150, 0};    // force
Eigen::Vector3d gravity = {0, 0, -9.8};

double groundMu = 0.5;
double ballMu = 1.0;
double boxMu = 0.8;

}

#endif //BENCHMARK_ROLLING_HPP

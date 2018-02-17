//
// Created by kangd on 17.02.18.
//

#ifndef BENCHMARK_THOUSAND_HPP
#define BENCHMARK_THOUSAND_HPP

#include <Eigen/Geometry>

namespace benchmark {

// parameters
double dt = 0.01;

double gap = 1.1;
double perturbation = 0.001;
double dropHeight = 5.0;

// object geometry
double ballR = 0.5;
double ballM = 1500;

Eigen::Vector3d boxSize = {0.8, 0.4, 0.2};
double boxM = 170;

Eigen::Vector2d capsuleSize = {0.2, 0.6};
double capsuleM = 300;

}

#endif //BENCHMARK_THOUSAND_HPP

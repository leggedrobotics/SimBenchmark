//
// Created by kangd on 19.03.18.
//

#ifndef DARTSIM_CHECKERBOARD_HPP
#define DARTSIM_CHECKERBOARD_HPP

#include "SingleBodyObject.hpp"

namespace dart_sim {
namespace object {

class CheckerBoard: public SingleBodyObject {

 public:
  CheckerBoard(double xLength, double yLength);

};

} // object
} // dart_sim

#endif //BENCHMARK_CHECKERBOARD_HPP

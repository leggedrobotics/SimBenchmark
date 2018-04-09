//
// Created by kangd on 09.04.18.
//

#ifndef BENCHMARK_CHECKERBOARDINTERFACE_HPP
#define BENCHMARK_CHECKERBOARDINTERFACE_HPP

namespace benchmark {
namespace object {

enum CheckerboardShape {
  PLANE_SHAPE,
  BOX_SHAPE
};

enum CheckerBoardOption {
  GRID = 1<<(1),
};

class CheckerboardInterface {

 protected:
  CheckerboardShape checkerboardShape = PLANE_SHAPE;

};


} // object

} // benchmark

#endif //BENCHMARK_CHECKERBOARDINTERFACE_HPP

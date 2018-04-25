//
// Created by kangd on 19.03.18.
//

#ifndef DARTSIM_CHECKERBOARD_HPP
#define DARTSIM_CHECKERBOARD_HPP

#include "common/interface/CheckerboardInterface.hpp"
#include "DartSingleBodyObject.hpp"

namespace dart_sim {
namespace object {

class DartCheckerBoard: public DartSingleBodyObject,
                        public benchmark::object::CheckerboardInterface {

 public:
  DartCheckerBoard(double xLength, double yLength, benchmark::object::CheckerboardShape shape, int id);

  virtual void getPosition_W(benchmark::Vec<3>& pos_w) override ;
  virtual void getQuaternion(benchmark::Vec<4>& quat) override ;

};

} // object
} // dart_sim

#endif //BENCHMARK_CHECKERBOARD_HPP

//
// Created by kangd on 28.05.18.
//

#ifndef BENCHMARK_BTMBBOX_HPP
#define BENCHMARK_BTMBBOX_HPP

#include "BtMbSingleBodyObject.hpp"

namespace bullet_mb_sim {
namespace object {

class BtMbBox: public BtMbSingleBodyObject {

 public:
  BtMbBox(double xlength,
          double ylength,
          double zlength,
          double mass,
          b3RobotSimulatorClientAPI_NoGUI *api);

};

} // object
} // bullet_mb_sim

#endif //BENCHMARK_BTMBBOX_HPP

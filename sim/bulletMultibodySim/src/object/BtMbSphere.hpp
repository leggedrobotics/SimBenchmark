//
// Created by kangd on 28.05.18.
//

#ifndef BENCHMARK_BTMBSPHERE_HPP
#define BENCHMARK_BTMBSPHERE_HPP

#include "BtMbSingleBodyObject.hpp"

namespace bullet_mb_sim {
namespace object {

class BtMbSphere: public BtMbSingleBodyObject {

 public:
  BtMbSphere(double radius, double mass, b3RobotSimulatorClientAPI_NoGUI *api);

};

} // object
} // bullet_mb_sim

#endif //BENCHMARK_BTMBSPHERE_HPP

//
// Created by kangd on 27.05.18.
//

#ifndef BENCHMARK_BTMBCHECKERBOARD_HPP
#define BENCHMARK_BTMBCHECKERBOARD_HPP

#include "BtMbSingleBodyObject.hpp"

namespace bo = benchmark::object;

namespace bullet_mb_sim {
namespace object {

class BtMbCheckerBoard: public BtMbSingleBodyObject,
                        public bo::CheckerboardInterface
{

 public:
  BtMbCheckerBoard(double xLength,
                   double yLength,
                   b3RobotSimulatorClientAPI_NoGUI *api,
                   bo::CheckerboardShape shape);

  void getPosition_W(benchmark::Vec<3> &pos_w) override;
  void getQuaternion(benchmark::Vec<4> &quat) override;

};

}
}

#endif //BENCHMARK_BTMBCHECKERBOARD_HPP

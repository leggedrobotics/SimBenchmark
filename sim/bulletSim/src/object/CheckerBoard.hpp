//
// Created by kangd on 10.02.18.
//

#ifndef BULLETSIM_CHECKERBOARD_HPP
#define BULLETSIM_CHECKERBOARD_HPP

#include "common/interface/CheckerboardInterface.hpp"
#include "SingleBodyObject.hpp"

namespace bo = benchmark::object;

namespace bullet_sim {
namespace object {

class CheckerBoard: public SingleBodyObject,
                    public bo::CheckerboardInterface {

 public:
  CheckerBoard(double xLength, double yLength, bo::CheckerboardShape shape = bo::PLANE_SHAPE);

};

} // object
} // bullet_sim

#endif //BULLETSIM_CHECKERBOARD_HPP

//
// Created by kangd on 12.02.18.
//

#ifndef ODESIM_CHECKERBOARD_HPP
#define ODESIM_CHECKERBOARD_HPP

#include "SingleBodyObject.hpp"

namespace ode_sim {
namespace object {

class CheckerBoard: public SingleBodyObject {

 public:
  CheckerBoard(dWorldID worldId);
  virtual ~CheckerBoard();

};

} // object
} // ode_sim

#endif //ODESIM_CHECKERBOARD_HPP

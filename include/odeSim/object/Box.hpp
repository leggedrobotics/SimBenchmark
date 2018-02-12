//
// Created by kangd on 11.02.18.
//

#ifndef ODESIM_BOX_HPP
#define ODESIM_BOX_HPP

#include "SingleBodyObject.hpp"

namespace ode_sim {
namespace object {

class Box: public SingleBodyObject {

 public:
  Box(double xlength,
      double ylength,
      double zlength,
      double mass,
      dWorldID worldId);
  virtual ~Box();

};

} // object
} // ode_sim

#endif //ODESIM_BOX_HPP

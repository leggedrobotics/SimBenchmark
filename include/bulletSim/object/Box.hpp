//
// Created by kangd on 09.02.18.
//

#ifndef BULLETSIM_BOX_HPP
#define BULLETSIM_BOX_HPP

#include "SingleBodyObject.hpp"

namespace bullet_sim {
namespace object {

class Box: public SingleBodyObject {

 public:
  Box(double xlength, double ylength, double zlength, double mass);

};

} // object
} // bullet_sim

#endif //BULLETSIM_BOX_HPP

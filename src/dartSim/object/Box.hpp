//
// Created by kangd on 19.03.18.
//

#ifndef DARTSIM_BOX_HPP
#define DARTSIM_BOX_HPP

#include "SingleBodyObject.hpp"

namespace dart_sim {
namespace object {

class Box: public SingleBodyObject {

 public:
  Box(double xlength, double ylength, double zlength, double mass);

};

} // object
} // dart_sim

#endif //DARTSIM_BOX_HPP

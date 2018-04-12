//
// Created by kangd on 19.03.18.
//

#ifndef DARTSIM_BOX_HPP
#define DARTSIM_BOX_HPP

#include "DartSingleBodyObject.hpp"

namespace dart_sim {
namespace object {

class DartBox: public DartSingleBodyObject {

 public:
  DartBox(double xlength, double ylength, double zlength, double mass, int id);

};

} // object
} // dart_sim

#endif //DARTSIM_BOX_HPP

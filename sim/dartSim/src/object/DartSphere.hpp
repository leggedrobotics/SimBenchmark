//
// Created by kangd on 19.03.18.
//

#ifndef DARTSIM_SPHERE_HPP
#define DARTSIM_SPHERE_HPP

#include "DartSingleBodyObject.hpp"

namespace dart_sim {
namespace object {

class DartSphere: public DartSingleBodyObject {

 public:
  DartSphere(double radius, double mass, int id);

};

}
}

#endif //DARTSIM_SPHERE_HPP

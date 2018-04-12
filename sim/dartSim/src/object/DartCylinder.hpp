//
// Created by kangd on 04.04.18.
//

#ifndef BENCHMARK_CYLINDER_HPP
#define BENCHMARK_CYLINDER_HPP

#include "DartSingleBodyObject.hpp"

namespace dart_sim {
namespace object {

class DartCylinder: public DartSingleBodyObject {

 public:
  DartCylinder(double radius,
               double height,
               double mass,
               int id);

};

}
}

#endif //BENCHMARK_CYLINDER_HPP

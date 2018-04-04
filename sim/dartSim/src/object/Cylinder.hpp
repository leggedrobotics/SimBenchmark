//
// Created by kangd on 04.04.18.
//

#ifndef BENCHMARK_CYLINDER_HPP
#define BENCHMARK_CYLINDER_HPP

#include "SingleBodyObject.hpp"

namespace dart_sim {
namespace object {

class Cylinder: public SingleBodyObject {

 public:
  Cylinder(double radius, double height, double mass);

};

}
}

#endif //BENCHMARK_CYLINDER_HPP

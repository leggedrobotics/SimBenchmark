//
// Created by kangd on 15.02.18.
//

#ifndef BENCHMARK_CAPSULE_HPP
#define BENCHMARK_CAPSULE_HPP

#include "SingleBodyObject.hpp"

namespace bullet_sim {
namespace object {

class Capsule: public SingleBodyObject {

 public:
  Capsule(double radius, double height, double mass);

};

} // object
} // bullet_sim

#endif //BENCHMARK_CAPSULE_HPP

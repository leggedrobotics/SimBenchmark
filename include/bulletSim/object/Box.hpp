//
// Created by kangd on 09.02.18.
//

#ifndef BENCHMARK_BULLETBOX_HPP
#define BENCHMARK_BULLETBOX_HPP

#include "SingleBodyObject.hpp"

namespace bullet_sim {
namespace object {

class Box: public SingleBodyObject {

 public:
  Box(double xlength, double ylength, double zlength, double mass);

};

} // object
} // bullet_sim

#endif //BENCHMARK_BULLETBOX_HPP

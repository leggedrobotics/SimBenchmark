//
// Created by kangd on 04.04.18.
//

#ifndef DARTSIM_ARTICULATEDSYSTEM_HPP
#define DARTSIM_ARTICULATEDSYSTEM_HPP

#include <common/interface/ArticulatedSystemInterface.hpp>
#include "Object.hpp"

namespace dart_sim {
namespace object {

class ArticulatedSystem: public Object,
                         public benchmark::object::ArticulatedSystemInterface {

};

} // object
} // dart_sim

#endif //DARTSIM_ARTICULATEDSYSTEM_HPP

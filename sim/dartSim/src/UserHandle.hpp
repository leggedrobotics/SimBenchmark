//
// Created by jhwangbo on 02/02/18.
//

#ifndef DARTSIM_USERHANDLE_HPP
#define DARTSIM_USERHANDLE_HPP

#include <raiGraphics/RAI_graphics.hpp>
#include <common/UserHandle.hpp>

#include "object/DartSingleBodyObject.hpp"
#include "object/DartArticulatedSystem.hpp"

namespace dart_sim {

typedef benchmark::UserObjectHandle<dart_sim::object::DartSingleBodyObject> SingleBodyHandle;
typedef benchmark::UserObjectHandle<dart_sim::object::DartArticulatedSystem> ArticulatedSystemHandle;
//typedef UserObjectHandle<bullet_sim::object::Compound> CompoundHandle;
//typedef UserWireHandle<bullet_sim::Wire> WireHandle;
//typedef UserWireHandle<bullet_sim::StiffWire> StiffWireHandle;
//typedef UserWireHandle<bullet_sim::CompliantWire> CompliantWireHandle;

} // benchmark

#endif //DART_USERHANDLE_HPP

//
// Created by jhwangbo on 02/02/18.
//

#ifndef DARTSIM_USERHANDLE_HPP
#define DARTSIM_USERHANDLE_HPP

#include <raiGraphics/RAI_graphics.hpp>
#include <common/UserHandle.hpp>

#include "object/SingleBodyObject.hpp"
#include "object/ArticulatedSystem.hpp"

namespace dart_sim {

typedef benchmark::UserObjectHandle<dart_sim::object::SingleBodyObject> SingleBodyHandle;
typedef benchmark::UserObjectHandle<dart_sim::object::ArticulatedSystem> ArticulatedSystemHandle;
//typedef UserObjectHandle<bullet_sim::object::Compound> CompoundHandle;
//typedef UserWireHandle<bullet_sim::Wire> WireHandle;
//typedef UserWireHandle<bullet_sim::StiffWire> StiffWireHandle;
//typedef UserWireHandle<bullet_sim::CompliantWire> CompliantWireHandle;

} // benchmark

#endif //DART_USERHANDLE_HPP

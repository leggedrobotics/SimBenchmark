//
// Created by jhwangbo on 02/02/18.
//

#ifndef BULLETSIM_USERHANDLE_HPP
#define BULLETSIM_USERHANDLE_HPP

#include <raiGraphics/RAI_graphics.hpp>
#include <common/UserHandle.hpp>

#include "object/BtSingleBodyObject.hpp"
#include "object/ArticulatedSystem/BtArticulatedSystem.hpp"

namespace bullet_sim {

typedef benchmark::UserObjectHandle<bullet_sim::object::BtSingleBodyObject> SingleBodyHandle;
typedef benchmark::UserObjectHandle<bullet_sim::object::BtArticulatedSystem> ArticulatedSystemHandle;
//typedef UserObjectHandle<bullet_sim::object::Compound> CompoundHandle;
//typedef UserWireHandle<bullet_sim::Wire> WireHandle;
//typedef UserWireHandle<bullet_sim::StiffWire> StiffWireHandle;
//typedef UserWireHandle<bullet_sim::CompliantWire> CompliantWireHandle;

} // benchmark

#endif //BENCHMARK_USERHANDLE_HPP

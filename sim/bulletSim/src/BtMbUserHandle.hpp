//
// Created by jhwangbo on 02/02/18.
//

#ifndef BULLETMULTIBODYSIM_USERHANDLE_HPP
#define BULLETMULTIBODYSIM_USERHANDLE_HPP

#include <raiGraphics/RAI_graphics.hpp>
#include <common/UserHandle.hpp>

#include "object/BtMbArticulatedSystem.hpp"

namespace bullet_mb_sim {

//typedef benchmark::UserObjectHandle<bullet_sim::object::BtSingleBodyObject> SingleBodyHandle;
typedef benchmark::UserObjectHandle<bullet_mb_sim::object::BtMbArticulatedSystem> ArticulatedSystemHandle;
//typedef UserObjectHandle<bullet_sim::object::Compound> CompoundHandle;
//typedef UserWireHandle<bullet_sim::Wire> WireHandle;
//typedef UserWireHandle<bullet_sim::StiffWire> StiffWireHandle;
//typedef UserWireHandle<bullet_sim::CompliantWire> CompliantWireHandle;

} // bullet_mb_sim

#endif //BULLETMULTIBODYSIM_USERHANDLE_HPP

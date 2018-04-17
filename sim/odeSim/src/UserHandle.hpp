//
// Created by jhwangbo on 02/02/18.
//

#ifndef ODESIM_USERHANDLE_HPP
#define ODESIM_USERHANDLE_HPP

#include <raiGraphics/RAI_graphics.hpp>
#include <common/UserHandle.hpp>

#include "object/OdeSingleBodyObject.hpp"
#include "object/ArticulatedSystem/OdeArticulatedSystem.hpp"

namespace ode_sim {

typedef benchmark::UserObjectHandle<ode_sim::object::OdeSingleBodyObject> SingleBodyHandle;
typedef benchmark::UserObjectHandle<ode_sim::object::OdeArticulatedSystem> ArticulatedSystemHandle;
//typedef UserObjectHandle<bullet_sim::object::Compound> CompoundHandle;
//typedef UserWireHandle<bullet_sim::Wire> WireHandle;
//typedef UserWireHandle<bullet_sim::StiffWire> StiffWireHandle;
//typedef UserWireHandle<bullet_sim::CompliantWire> CompliantWireHandle;

} // ode_sim

#endif //ODESIM_USERHANDLE_HPP

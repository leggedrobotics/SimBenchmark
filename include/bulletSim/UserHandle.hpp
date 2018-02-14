//
// Created by jhwangbo on 02/02/18.
//

#ifndef BULLETSIM_USERHANDLE_HPP
#define BULLETSIM_USERHANDLE_HPP

#include "interface/UserHandle.hpp"
#include "bulletSim/object/SingleBodyObject.hpp"

namespace bullet_sim {

typedef benchmark::UserObjectHandle<bullet_sim::object::SingleBodyObject> SingleBodyHandle;
//typedef benchmark::UserObjectHandle<bullet_sim::object::ArticulatedSystem> ArticulatedSystemHandle;
//typedef UserObjectHandle<bullet_sim::object::Compound> CompoundHandle;
//typedef UserWireHandle<bullet_sim::Wire> WireHandle;
//typedef UserWireHandle<bullet_sim::StiffWire> StiffWireHandle;
//typedef UserWireHandle<bullet_sim::CompliantWire> CompliantWireHandle;

}
#endif // BULLETSIM_USERHANDLE_HPP

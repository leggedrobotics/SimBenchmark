//
// Created by kangd on 19.02.18.
//

#ifndef MUJOCOSIM_USERHANDLE_HPP
#define MUJOCOSIM_USERHANDLE_HPP

#include "base/UserHandle.hpp"
#include "object/SingleBodyObject.hpp"

namespace bullet_sim {

typedef benchmark::UserObjectHandle<mujoco_sim::object::SingleBodyObject> SingleBodyHandle;
//typedef benchmark::UserObjectHandle<bullet_sim::object::ArticulatedSystem> ArticulatedSystemHandle;
//typedef UserObjectHandle<bullet_sim::object::Compound> CompoundHandle;
//typedef UserWireHandle<bullet_sim::Wire> WireHandle;
//typedef UserWireHandle<bullet_sim::StiffWire> StiffWireHandle;
//typedef UserWireHandle<bullet_sim::CompliantWire> CompliantWireHandle;

}

#endif //MUJOCOSIM_USERHANDLE_HPP

//
// Created by jhwangbo on 02/02/18.
//

#ifndef BENCHMARK_USERHANDLE_HPP
#define BENCHMARK_USERHANDLE_HPP

#include <raiGraphics/RAI_graphics.hpp>

#include "SingleBodyObject.hpp"
#include "ArticulatedSystem.hpp"

namespace benchmark {

template<typename S>
class UserHandle {
 public:

  UserHandle(S* s) : s_(s){};

  S* operator ->() {
    return s_;
  }

  operator S*() {
    return s_;
  }

  bool operator ==(const UserHandle<S>& rhs) {
    return rhs.s_ == s_;
  }

  S* s_;
  bool hidable = true;
};

template<typename S>
class UserObjectHandle : public UserHandle<S> {
  friend class World_RG;

  std::vector<rai_graphics::object::SingleBodyObject*> g_, ag_;

 public:
  UserObjectHandle(S* s, std::vector<rai_graphics::object::SingleBodyObject*> g, std::vector<rai_graphics::object::SingleBodyObject*> ag): UserHandle<S>(s), g_(g), ag_(ag){}

  std::vector<rai_graphics::object::SingleBodyObject*>& visual() {
    return g_;
  }

  std::vector<rai_graphics::object::SingleBodyObject*>& alternateVisual() {
    return ag_;
  }
};

template<typename S>
class UserWireHandle : public UserHandle<S> {
  friend class World_RG;

  std::vector<rai_graphics::object::Lines*> g_;

 public:
  UserWireHandle(S* s, std::vector<rai_graphics::object::Lines*> g) : UserHandle<S>(s), g_(g) {}

  std::vector<rai_graphics::object::Lines*>& visual() {
    return g_;
  }
};

typedef benchmark::UserObjectHandle<benchmark::object::SingleBodyObject> SingleBodyHandle;
typedef benchmark::UserObjectHandle<benchmark::object::ArticulatedSystem> ArticulatedSystemHandle;
//typedef UserObjectHandle<bullet_sim::object::Compound> CompoundHandle;
//typedef UserWireHandle<bullet_sim::Wire> WireHandle;
//typedef UserWireHandle<bullet_sim::StiffWire> StiffWireHandle;
//typedef UserWireHandle<bullet_sim::CompliantWire> CompliantWireHandle;

} // benchmark

#endif //BENCHMARK_USERHANDLE_HPP

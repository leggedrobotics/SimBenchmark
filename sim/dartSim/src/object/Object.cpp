//
// Created by kangd on 04.04.18.
//

#include <boost/shared_ptr.hpp>
#include "Object.hpp"

dart_sim::object::Object::~Object() {
}

const std::shared_ptr <dart::dynamics::Skeleton> &dart_sim::object::Object::getSkeletonPtr() {
  return skeletonPtr_;
}

//
// Created by kangd on 04.04.18.
//

#include <boost/shared_ptr.hpp>
#include "DartObject.hpp"

dart_sim::object::DartObject::~DartObject() {
}

const std::shared_ptr <dart::dynamics::Skeleton> &dart_sim::object::DartObject::getSkeletonPtr() {
  return skeletonPtr_;
}

//
// Created by kangd on 04.04.18.
//

#include <boost/shared_ptr.hpp>
#include "Object.hpp"

const std::shared_ptr <dart::dynamics::Skeleton> &dart_sim::object::Object::getSkeletonPtr() const {
  return skeletonPtr_;
}

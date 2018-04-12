//
// Created by kangd on 04.04.18.
//

#ifndef DARTSIM_OBJECT_HPP
#define DARTSIM_OBJECT_HPP

#include <dart/dart.hpp>

namespace dart_sim {
namespace object {

class DartObject {

 public:
  virtual ~DartObject();

  const virtual dart::dynamics::SkeletonPtr &getSkeletonPtr();

 protected:
  dart::dynamics::SkeletonPtr skeletonPtr_;

};

} // object
} // dart_sim

#endif //DARTSIM_OBJECT_HPP

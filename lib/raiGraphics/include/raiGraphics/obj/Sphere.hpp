//
// Created by jhwangbo on 01.12.16.
//

#ifndef RAI_SPHERE_HPP
#define RAI_SPHERE_HPP
#include "SingleBodyObject.hpp"
#include <vector>

namespace rai_graphics {
namespace object {

class Sphere : public SingleBodyObject {

 public:
  Sphere(float radius, bool isSelectable=false);
  float radius_;

};

} // object
} // rai_graphics

#endif //RAI_SPHERE_HPP

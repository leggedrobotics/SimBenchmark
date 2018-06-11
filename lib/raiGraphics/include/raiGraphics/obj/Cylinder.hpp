//
// Created by joonho on 19.05.17.
//

#ifndef RAI_CYLINDER_HPP
#define RAI_CYLINDER_HPP

#include "SingleBodyObject.hpp"
#include <vector>

namespace rai_graphics {
namespace object {

class Cylinder: public SingleBodyObject {

 public:
  Cylinder(float r, float l, bool isSelectable=true);

 private:

};

} // object
} // rai_graphics
#endif //RAI_CYLINDER_HPP

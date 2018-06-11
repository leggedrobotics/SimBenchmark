//
// Created by joonho on 19.05.17.
//

#ifndef RAI_CONE_HPP
#define RAI_CONE_HPP

#include "SingleBodyObject.hpp"
#include <vector>

namespace rai_graphics {
namespace object {

class Cone: public SingleBodyObject {

 public:
  Cone(float r, float l, bool isSelectable=true);

 private:

};

} // object
} // rai_graphics

#endif //RAI_CONE_HPP

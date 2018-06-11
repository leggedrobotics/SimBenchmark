//
// Created by kangd on 29.11.17.
//

#ifndef RAIGRAPHICSOPENGL_CAPSULE_HPP
#define RAIGRAPHICSOPENGL_CAPSULE_HPP

#include "SingleBodyObject.hpp"
#include <vector>

namespace rai_graphics {
namespace object {

class Capsule: public SingleBodyObject {
 public:
  /// total height of capsule is (2*r + l)
  Capsule(float r, float l, bool isSelectable=true);

};

} // object
} // rai_graphics

#endif //RAIGRAPHICSOPENGL_CAPSULE_HPP

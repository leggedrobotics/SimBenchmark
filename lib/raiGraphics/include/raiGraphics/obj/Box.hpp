//
// Created by kangd on 16.10.17.
//

#ifndef RAIGRAPHICSOPENGL_BOX_HPP
#define RAIGRAPHICSOPENGL_BOX_HPP

#include "SingleBodyObject.hpp"

namespace rai_graphics {
namespace object {

class Box: public SingleBodyObject {
 public:
  Box(float xLength, float yLength, float zLength, bool isSelectable=false);

 private:

};

} // object
} // rai_graphics



#endif //RAIGRAPHICSOPENGL_BOX_HPP

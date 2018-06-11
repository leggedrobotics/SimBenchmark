//
// Created by jhwangbo on 01.05.17.
//

#ifndef RAI_CHECKERBOARD_HPP
#define RAI_CHECKERBOARD_HPP

#include "SingleBodyObject.hpp"

namespace rai_graphics {
namespace object {

class CheckerBoard : public SingleBodyObject {

 public:

  CheckerBoard(float gridSize, float width, float length, float reflectanceI, std::vector<float> color1={0,0,0}, std::vector<float> color2={1,1,1});
  void drawGridLines();
  void setGridLineWidth(float width);
  void setBoardColor(std::vector<float> firstColor, std::vector<float> secondColor);

  glm::vec3 col1, col2;
  float gridSize_;
  float width, length;
  bool gridMode = false;
  float gridLineWidth=1.0f;
};

} // object
} // rai_graphics


#endif //RAI_CHECKERBOARD_HPP

//
// Created by kangd on 16.10.17.
//

#ifndef RAIGRAPHICSOPENGL_RECTANGLE_HPP
#define RAIGRAPHICSOPENGL_RECTANGLE_HPP

#include <SDL2/SDL_ttf.h>
#include "SingleBodyObject.hpp"

namespace rai_graphics {
namespace object {

class Rectangle: public SingleBodyObject {
 public:
  Rectangle(float windowWidth, float windowHeight);
  ~Rectangle();

  void setTranslation(float xpos, float ypos);

  void setSize(float xsize, float ysize);

  void setTextWrap(int tw);

  void writeText(const std::vector<TTF_Font *>& font, const std::string& txt);

  /// from 1-5
  void setFontSize(int size);

  void bindTexture();

  float sizeX_ = 0, sizeY_ = 0;
  float posX_ = 0, posY_ = 0;
  float windowWidth_, windowHeight_;

 private:
  GLuint tex_;
  bool isTextured = false;
  SDL_Surface* surf;
  int wrapLength = 200;
  TTF_Font *font;
  int fontSize=3;
};

} // object
} // rai_graphics



#endif //RAIGRAPHICSOPENGL_BOX_HPP

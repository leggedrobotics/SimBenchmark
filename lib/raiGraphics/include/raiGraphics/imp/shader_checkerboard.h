#ifndef SHADER_CHECKERBOARD_INCLUDED_H
#define SHADER_CHECKERBOARD_INCLUDED_H

#include <raiGraphics/obj/CheckerBoard.hpp>
#include "shader.hpp"
#include "raiCommon/rai_utils.hpp"

namespace rai_graphics {

class Shader_checkerboard : public Shader {
 public:
  Shader_checkerboard();
  ~Shader_checkerboard();

  virtual std::string shaderFileName();
  void Bind();
  void UnBind();
  void Update(Camera *camera,  Light *light, object::CheckerBoard* obj);
  void Update(Camera *camera,  Light *light, object::SingleBodyObject* obj);
  void UpdateForReflection(Camera *camera, Light *light, object::SingleBodyObject* obj, object::CheckerBoard* chk);

 protected:
 private:
  static const unsigned int NUM_SHADERS = 2;
  static const unsigned int NUM_UNIFORMS = 4;

  GLuint m_shaders[NUM_SHADERS];
  GLuint m_uniforms[NUM_UNIFORMS];
};

} // rai_graphics

#endif

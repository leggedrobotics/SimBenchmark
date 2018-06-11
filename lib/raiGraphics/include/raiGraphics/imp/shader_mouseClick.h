#ifndef SHADER_MOUSECLICK_INCLUDED_H
#define SHADER_MOUSECLICK_INCLUDED_H

#include "shader.hpp"

namespace rai_graphics {

class Shader_mouseClick : public Shader {
 public:
  Shader_mouseClick();
  ~Shader_mouseClick();

  virtual std::string shaderFileName();
  void Bind();
  void UnBind();
  void Update(Camera *camera,  Light *light, object::SingleBodyObject* obj);
  void Update(Camera *camera,  object::SingleBodyObject* obj);
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

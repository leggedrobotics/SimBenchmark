#ifndef SHADER_LINE_INCLUDED_H
#define SHADER_LINE_INCLUDED_H

#include "shader.hpp"

namespace rai_graphics {

class Shader_line : public Shader {
 public:
  Shader_line();
  ~Shader_line();

  virtual std::string shaderFileName();
  void Bind();
  void UnBind();
  void Update(Camera *camera, Light *light, std::vector<float>& clr);
  void Update(Camera *camera, Light *light, object::SingleBodyObject* obj){}
  void UpdateForReflection(Camera *camera, Light *light, object::SingleBodyObject* obj, object::CheckerBoard* chk){};

 protected:
 private:
  static const unsigned int NUM_SHADERS = 2;

  GLuint m_shaders[NUM_SHADERS];
};

} // rai_graphics

#endif

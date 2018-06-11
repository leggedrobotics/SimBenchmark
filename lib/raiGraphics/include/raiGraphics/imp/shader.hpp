//
// Created by jhwangbo on 17. 4. 30.
//

#ifndef RAI_SHADER_HPP
#define RAI_SHADER_HPP

#include <string>
#include <GL/glew.h>
#include "transform.h"
#include "Light.hpp"
#include "raiGraphics/obj/SingleBodyObject.hpp"
#include "raiGraphics/obj/CheckerBoard.hpp"
#include "camera.h"
#include <iostream>
#include <fstream>

namespace rai_graphics {

class Shader {

 public:
  virtual void Bind() = 0;
  virtual void UnBind() = 0;
  virtual void Update(Camera *camera, Light *light, object::SingleBodyObject* obj) = 0;
  virtual void UpdateForReflection(Camera *camera, Light *light, object::SingleBodyObject* obj, object::CheckerBoard* chk) = 0;

 protected:
  std::string LoadShader(const std::string &fileName);
  void CheckShaderError(GLuint shader, GLuint flag, bool isProgram, const std::string &errorMessage);
  GLuint CreateShader(const std::string &text, unsigned int type);
  GLuint m_program;

};

} // rai_graphics

#endif //RAI_SHADER_HPP

#ifndef TRANSFORM_INCLUDED_H
#define TRANSFORM_INCLUDED_H

#include <glm/glm.hpp>
#include "glm/gtx/quaternion.hpp"
#include <glm/gtx/transform.hpp>

namespace rai_graphics {

struct Transform {
 public:
  Transform(const glm::vec3 &pos = glm::vec3(),
            const glm::quat &rot = glm::quat(),
            const glm::vec3 &scale = glm::vec3(1.0f, 1.0f, 1.0f)) {
    this->pos = pos;
    this->rot = rot;
    this->scale = scale;
  }

  glm::mat4 GetModel() const {
    glm::mat4 rotMat = glm::toMat4(rot);
    glm::mat4 posMat = glm::translate(pos);

    return posMat * rotMat;
  }

  inline glm::mat4 GetM() const {
    glm::mat4 M = GetModel();
    return M;//camera.GetViewProjection() * GetModel(); VPM
  }

  inline glm::vec3 *GetPos() { return &pos; }
  inline glm::quat *GetRot() { return &rot; }
  inline glm::vec3 *GetScale() { return &scale; }

  inline void SetPos(glm::vec3 &posL) { pos = posL; }
  inline void SetRot(glm::quat &rotL) { rot = rotL; }
  inline void SetScale(glm::vec3 &scaleL) { scale = scaleL; }
 protected:
 private:
  glm::vec3 pos = {0,0,0};
  glm::quat rot = {1,0,0,0};
  glm::vec3 scale = {1,1,1};
};

} // rai_graphics

#endif

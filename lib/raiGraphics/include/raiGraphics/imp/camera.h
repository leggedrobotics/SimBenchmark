#ifndef CAMERA_INCLUDED_H
#define CAMERA_INCLUDED_H

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <cmath>
#include <iostream>
#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <raiGraphics/obj/SingleBodyObject.hpp>
#include "vector3d.h"

namespace rai_graphics {
struct Camera {
 public:

  Camera(const glm::vec3 &pos, float fov, float aspect, float zNear, float zFar);
  void update();
  void GetVP(glm::mat4& vp);
  void GetPose(glm::mat4 &vp);
  void GetPos(glm::vec3& position);
  void Control(SDL_Event e, bool stayAboveZero);
  void follow(rai_graphics::object::SingleBodyObject* obj, Eigen::Vector3d pos);
  void follow(rai_graphics::object::SingleBodyObject* obj);
  void zoomIn();
  void zoomOut();
  object::SingleBodyObject* getToFollowObj();
  void unFollowOb();

 protected:
 private:

  void stayHere();

  object::SingleBodyObject* toFollowObj = nullptr;
  glm::vec4 relativePos;
  glm::mat4 vp_, pose_;

  vector3d loc;
  float camPitch, camYaw;
  float camAngularSpeed;
  float camLinearSpeed = 0.1;

  bool mi = true;
  void lockCamera();
  bool mousePressedLastTimeStep = false;
  bool mouseRightPressedLastTimeStep = false;

  int prevMx = -1, prevMy = -1;
  std::mutex mtx;
  glm::mat4 projection;
  glm::vec3 pos;
  glm::vec3 forward;
  glm::vec3 up;
  const Uint8 *keyState;
  unsigned switchTime = 0;

};

} // rai_graphics

#endif

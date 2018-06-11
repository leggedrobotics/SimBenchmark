//
// Created by jhwangbo on 17. 5. 1.
//

#ifndef RAI_SUPEROBJECT_HPP
#define RAI_SUPEROBJECT_HPP

#include "SingleBodyObject.hpp"
#include "raiGraphics/imp/shader.hpp"
#include "raiGraphics/imp/shader_basic.h"
#include "raiGraphics/obj/Object.hpp"
#include "raiCommon/TypeDef.hpp"

namespace rai_graphics {
namespace object {

class MultiBodyObject: public Object {

 public:

  virtual void init() = 0;
  virtual void destroy() = 0;

  void draw(Camera *camera,  Light *light, float transparency, CheckerBoard* chk = nullptr);
  void setVisibility(bool visibility) {visible = visibility;}
  bool isVisible() {return visible;}
  void showGhosts(int maxGhosts, float transparency);
  void addGhostsNow();
  std::vector<SingleBodyObject*> getChildren(){ return objs; }

  bool reflectable = true;
 protected:

  void setTrans(std::vector<Transform>& trans);
  void getTrans(std::vector<Transform>& trans);
  void turnOnGhost(bool ghostOn);
  void drawSnapshot(Camera *camera,  Light *light, float transparency, CheckerBoard* chk = nullptr);

  bool visible = true;
  std::vector<SingleBodyObject*> objs;
  std::vector<Shader*> shader;
  std::vector<std::vector<Transform> > ghosts;
  std::vector<Transform> currentPose;
  float ghostTransparency;
  int maxGhostN = 0;
  int oldestGhost = 0;

};

} // object
} // rai_graphics

#endif //RAI_SUPEROBJECT_HPP

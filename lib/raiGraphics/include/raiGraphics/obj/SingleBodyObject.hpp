//
// Created by jhwangbo on 17. 4. 28.
//

#ifndef PROJECT_OBJECT_HPP
#define PROJECT_OBJECT_HPP
#include <mutex>

#include "Eigen/Geometry"
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "raiGraphics/imp/transform.h"
#include "raiGraphics/imp/Light.hpp"
#include "raiGraphics/obj/Object.hpp"

namespace rai_graphics{
namespace object {

enum MeshBufferPositions {
  POSITION_VB,
  TEXCOORD_VB,
  NORMAL_VB,
  COLOR_VB,
  INDEX_VB
};

class SingleBodyObject: public Object {
 public:

  virtual void draw();

  virtual void init();

  virtual void destroy();

  void setPose(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternionAsVector);
  void setPose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotationMatrix);
  void setPose(const Eigen::Vector3d &position, const Eigen::Quaterniond &quat);
  void setPose(const Eigen::Matrix4d &ht);

  void setOri(const Eigen::Vector4d &quaternionAsVector);
  void setOri(const Eigen::Matrix3d &rotationMat);
  void setOri(const Eigen::Quaterniond &quat);

  void setPos(const Eigen::Vector3d &position);
  void setPos(double x, double y, double z);
  void setOri(double w, double x, double y, double z);

  void setTransform(Transform& trans);
  void setLightProp(std::vector<float>& amb, std::vector<float>& diff, std::vector<float>& spec, float shine);
  void setColor(std::vector<float> colorL);
  void setTransparency(float transparency);

  float getTransparency();

  glm::mat4 getScale();
  void getTransform(Transform& trans);

  void getColor(std::vector<float>& clr);
  void getLightPropAmb(std::vector<float>& amb);
  void getLightPropDiff(std::vector<float>& diff);
  void getLightPropSpec(std::vector<float>& spec);
  void getShiness(float& shine);
  const std::vector<glm::vec3>& getVertexPositions() const;
  bool isVisible() {return visible;}

  unsigned getSelectableObIndex() const {return selectableObIndex; };
  void setSelectableObIndex(unsigned idx) {selectableObIndex = idx; };

  void setVisibility(bool visibility) {visible = visibility;}
  void setScale(double scale);
  void setScale(double scale1,double scale2,double scale3);

  void setTempTransform(int i);

  void usingTempTransform(bool utt) { tempTransformOn = utt; };

  void mutexLock() {mtx.lock();}
  void mutexUnLock() {mtx.unlock();}

  void highlight();
  void deHighlight();

  virtual void addGhost(Eigen::Vector3d &position);
  virtual void addGhost(Eigen::Vector3d &position, Eigen::Quaterniond &quat);
  void addGhost(Eigen::Vector3d &position, Eigen::Quaterniond &quat, std::vector<float> color, std::vector<float> scale);
  void clearGhost();
  std::vector<Transform> & getGhosts();
  bool isSelectable() const;
  bool loadTexture();

  ShaderType defaultShader = object::RAI_SHADER_BASIC;

  bool reflectable = true;
  glm::vec3 com;
  bool hasColorCoord = false;

 protected:
  void registerToGPU();
  Transform transform;
  Transform transformGhost;
  bool tempTransformOn = false;
  glm::mat4 scaleMat_;
  glm::mat4 scaleMatGhost_;
  std::vector<float> color_ = {0.7, 0.7, 0.7};
  std::vector<float> colorGhost_ = {0.7, 0.7, 0.7};
  std::vector<float> amb_m = {0.6, 0.6, 0.6};
  std::vector<float> amb_m_orig = {0.6, 0.6, 0.6};
  std::vector<float> diff_m = {1.0,1.0,1.0};
  std::vector<float> spec_m = {0.6,0.6,0.6};
  float transparency_ = 1.0, transparencyOrig_;
  float shininess = 50;
  std::vector<glm::vec3> colorsCoords;
  bool visible = true;
  std::vector<glm::vec3> positions;
  std::vector<glm::vec2> texCoords;
  std::vector<glm::vec3> normals;
  std::vector<unsigned int> indices;
  static const unsigned int NUM_BUFFERS = 5;
  GLuint m_vertexArrayObject;
  GLuint m_vertexArrayBuffers[NUM_BUFFERS];
  unsigned int m_numIndices;
  std::mutex mtx;
  unsigned selectableObIndex;

//  void drawSnapshot(Camera *camera,  Light *light, float transparency);
  std::vector<Transform> ghosts;
  std::vector<glm::vec3> ghostColor;
  std::vector<glm::vec3> ghostScale;

//  Shader* shader = nullptr;

  bool selectable_ = false;

};

} // object
} // rai_graphics

#endif //PROJECT_OBJECT_HPP

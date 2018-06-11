//
// Created by joonho on 07.05.17.
//

#ifndef RAI_BIPED_HPP
#define RAI_BIPED_HPP
#include <raiCommon/TypeDef.hpp>
#include "MultiBodyObject.hpp"
#include "Mesh.hpp"
#include "Sphere.hpp"

namespace rai_graphics {
namespace object {

class Biped : public MultiBodyObject {

 public:

  Biped();
  ~Biped();
  void init();
  void destroy();
  void setPose(std::vector<rai::HomogeneousTransform> &bodyPose);
  SingleBodyObject* basePtr(){ return &base; }

 private:

  Mesh base;
  Mesh haa_l;
  Mesh hfe_l;
  Mesh thigh_l;
  Mesh shank_l;
  Mesh afe_l;

  Mesh haa_r;
  Mesh hfe_r;
  Mesh thigh_r;
  Mesh shank_r;
  Mesh afe_r;

  Mesh foot_l;
  Mesh foot_r;

  std::vector<rai::HomogeneousTransform> defaultPose_;
};

} // object
} // rai_graphics

#endif //RAI_BIPED_HPP

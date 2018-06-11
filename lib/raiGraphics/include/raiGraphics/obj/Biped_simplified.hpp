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

class Biped_simplified : public MultiBodyObject {

 public:

  Biped_simplified();
  ~Biped_simplified();
  void init();
  void destroy();
  void setPose(std::vector<rai::HomogeneousTransform> &bodyPose);
  SingleBodyObject* basePtr(){ return &base; }

  SingleBodyObject* footlPtr(){ return &foot_l; }
  SingleBodyObject* footrPtr(){ return &foot_r; }


 private:
  Mesh base;
  Mesh haa_l;
  Mesh hfe_l;
  Mesh thigh_l;
  Mesh shank_l;
//  Mesh afe_l;

  Mesh haa_r;
  Mesh hfe_r;
  Mesh thigh_r;
  Mesh shank_r;
//  Mesh afe_r;

  Sphere foot_l;
  Sphere foot_r;

  std::vector<rai::HomogeneousTransform> defaultPose_;
};

} // object
} // rai_graphics

#endif //RAI_BIPED_HPP

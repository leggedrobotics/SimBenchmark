//
// Created by jhwangbo on 17. 5. 3.
//

#ifndef RAI_ANYMAL_HPP
#define RAI_ANYMAL_HPP

#include <raiCommon/TypeDef.hpp>
#include "MultiBodyObject.hpp"
#include "Mesh.hpp"
#include "Sphere.hpp"

namespace rai_graphics {
namespace object {

class ANYmal : public MultiBodyObject {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ANYmal();
  ~ANYmal();
  void init();
  void destroy();
  void setPose(std::vector<rai::HomogeneousTransform> &bodyPose);
  SingleBodyObject* basePtr(){ return &base; }

 private:

  Mesh base;
  Mesh hip_lf;
  Mesh hip_rf;
  Mesh hip_lh;
  Mesh hip_rh;

  Mesh thigh_lf;
  Mesh thigh_rf;
  Mesh thigh_lh;
  Mesh thigh_rh;

  Mesh shank_lf;
  Mesh shank_rf;
  Mesh shank_lh;
  Mesh shank_rh;

  Mesh adapter_lf;
  Mesh adapter_rf;
  Mesh adapter_lh;
  Mesh adapter_rh;

  Sphere foot_lf;
  Sphere foot_rf;
  Sphere foot_lh;
  Sphere foot_rh;
  std::vector<rai::HomogeneousTransform> defaultPose_;
};

} // object
} // rai_graphics

#endif //RAI_ANYMAL_HPP

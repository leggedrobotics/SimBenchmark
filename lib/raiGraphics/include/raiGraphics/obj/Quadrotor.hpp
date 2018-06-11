//
// Created by jhwangbo on 17. 5. 3.
//

#ifndef RAI_QUADROTOR_HPP
#define RAI_QUADROTOR_HPP

#include <raiCommon/TypeDef.hpp>
#include "MultiBodyObject.hpp"
#include "Mesh.hpp"

namespace rai_graphics {
namespace object {

class Quadrotor : public MultiBodyObject {

 public:
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quadrotor(float scale);
  ~Quadrotor();
  void init();
  void destroy();
  void setPose(Eigen::Matrix4d &ht);
  void setPose(Eigen::Vector3d &position, Eigen::Vector4d &quat);
  void spinRotors();
  SingleBodyObject* basePtr(){ return &base; }

 private:

  Mesh base;
  Mesh prop1;
  Mesh prop2;
  Mesh prop3;
  Mesh prop4;
  Mesh brain;

  std::vector<rai::HomogeneousTransform> defaultPose_;
};

} // object
} // rai_graphics

#endif //RAI_QUADROTOR_HPP

//
// Created by kangd on 28.09.17.
//

#ifndef RAIGRAPHICSOPENGL_FRAME_HPP
#define RAIGRAPHICSOPENGL_FRAME_HPP

#include "MultiBodyObject.hpp"
#include "Arrow.hpp"
#include "raiCommon/math/RAI_math.hpp"
#include <vector>

namespace rai_graphics {
namespace object {

class CoordinateFrame : public MultiBodyObject {

 public:
  explicit CoordinateFrame(Eigen::Vector3d origin = {0.0, 0.0, 0.0},
                           float arrowBodyLength = 2.0,
                           float arrowHeadLength = 1.0,
                           float arrowBodyRadius = 0.25,
                           float arrowHeadRadius = 0.5);

  CoordinateFrame(Eigen::Vector3d &origin,
                  Eigen::Quaterniond &rotation,
                  float arrowBodyLength = 2.0,
                  float arrowHeadLength = 1.0,
                  float arrowBodyRadius = 0.25,
                  float arrowHeadRadius = 0.5);


  void setPose(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternionAsVectorWB);
  void setPose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotationMatrix);
  void setPose(const Eigen::Vector3d &position, const Eigen::Quaterniond &quaternionWB);
  void setPos(const Eigen::Vector3d &position);
  void setPos(double x, double y, double z);
  void setOri(const Eigen::Vector4d &quaternionAsVectorWB);
  void setOri(const Eigen::Matrix3d &rotationMatrixWB);
  void setOri(const Eigen::Quaterniond &quaternionWB);

  virtual ~CoordinateFrame();
  void init();
  void destroy();

 private:

  void initChildren();

  Eigen::Vector3d origin_;
  Eigen::Quaterniond rotation_;

  // arrow axis quternion w.r.t. world frame
  Eigen::Quaterniond WxAxisArrowQuaternion_;
  Eigen::Quaterniond WyAxisArrowQuaternion_;
  Eigen::Quaterniond WzAxisArrowQuaternion_;

  // arrow axis quaternion w.r.t. body frame
  const Eigen::Quaterniond BxAxisArrowQuaternion_ = {1, 0, 0, 0};
  const Eigen::Quaterniond ByAxisArrowQuaternion_ = {1/sqrt(2), 0, 0, 1/sqrt(2)};
  const Eigen::Quaterniond BzAxisArrowQuaternion_ = {1/sqrt(2), 0, -1/sqrt(2), 0};

  object::Arrow xAxisArrow_;
  object::Arrow yAxisArrow_;
  object::Arrow zAxisArrow_;

  float arrowBodyLength_;
  float arrowHeadLength_;
  float arrowBodyRadius_;
  float arrowHeadRadius_;
};

} // object
} // rai_graphics

#endif //RAIGRAPHICSOPENGL_FRAME_HPP

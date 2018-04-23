//
// Created by kangd on 23.04.18.
//

#include <Eigen/Geometry>
#include <raiCommon/utils/rai_message_logger/rai_message.hpp>

int main() {
  Eigen::Quaterniond quat1(0, 0, 0, 1);
  Eigen::Quaterniond quat2(0, 0, 0, 1);
  Eigen::Quaterniond result = quat1 * quat2;
  RAIINFO(result.w() << result.x() << result.y() << result.z());
}
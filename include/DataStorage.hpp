#pragma once

#include <Eigen/Geometry>

namespace iNav {

class IMUData {
public:
  double timestamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d acc;
};

class RefData {
public:
  double timestamp;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d att;
};

}  // namespace iNav

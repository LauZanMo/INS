#pragma once

#include <Eigen/Geometry>

namespace iNav {

class IMUData {
public:
  double timestamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d acc;
};

class IMUParam {
public:
  double VRW;
  double ARW;
  double gyro_bias_std;
  double T_gyro_bias;
  double acc_bias_std;
  double T_acc_bias;
  double gyro_scalar_std;
  double T_gyro_scalar;
  double acc_scalar_std;
  double T_acc_scalar;
};

class GnssData {
public:
  double timestamp;
  Eigen::Vector3d pos;
  Eigen::Vector3d pos_std;
};

class NavData {
public:
  double timestamp;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d att;
};

}  // namespace iNav

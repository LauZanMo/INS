#pragma once

#include <Eigen/Geometry>

using namespace Eigen;

namespace INS {

class IMUStorage {
public:
  double time_;
  double gyro_x_;
  double gyro_y_;
  double gyro_z_;
  double acc_x_;
  double acc_y_;
  double acc_z_;
};

class INSStorage {
public:
  double time_;
  double phi_;
  double lamda_;
  double h_;
  double v_n_;
  double v_e_;
  double v_d_;
  double roll_;
  double pitch_;
  double yaw_;
};

}  // namespace INS
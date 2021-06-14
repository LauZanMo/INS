/**
 * @file INS.hpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief INS class
 * @version 1.0
 * @date 2021-06-13
 * 
 * @copyright Copyright (c) 2021 WHU-Drones
 * 
 */
#pragma once

#include <Eigen/Geometry>

using namespace Eigen;

namespace INS {

struct InsOutput {
  Quaternion<double> theta;
  Vector3d velocity;
  Vector3d position;
};

class INS {
public:
  INS();
  ~INS();

  void SensorUpdate(double time, Vector3d delta_theta, Vector3d delta_velocity);
  InsOutput INSUpdate();

private:
  // IMU output
  Vector3d delta_theta_, last_delta_theta_;
  Vector3d delta_velocity_, last_delta_velocity_;

  // INS data
  double time_, last_time_, delta_time_;
  Quaternion<double> theta_, last_theta_;
  Vector3d velocity_, last_velocity_, before_last_velocity_;
  Vector3d position_, last_position_;
  double last_R_M_, last_R_N_;
  Vector3d last_omega_ie_, before_last_omega_ie_;
  Vector3d last_omega_en_, before_last_omega_en_;

  // constant
  const double a_ = 6378137.0;
  const double e_ = 0.08181919104;
  const double omega_e_ = 0.26179938779915;
  const Vector3d g_p_;

  // member function
  void AttitudeUpdate();
  void VelocityUpdate();
  void PositionUpdate();
  void DataRecord();

  inline Matrix3d ToSkewSymmetricMat(const Vector3d vec) {
    Matrix3d vec_cross;
    vec_cross << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return vec_cross;
  }
};

}  // namespace INS

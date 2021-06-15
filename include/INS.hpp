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
  Vector3d theta;
  Vector3d velocity;
  Vector3d position;
};

class INS {
public:
  INS(double init_time, Vector3d init_theta, Vector3d init_velocity,
      double init_phi, double init_lamda, double init_height,
      Vector3d init_delta_theta, Vector3d init_delta_velocity);
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
  const double pi_ = 3.1415926535897932384626433832795;
  const double rad2degree_ = 180.0 / pi_;
  const double degree2rad_ = pi_ / 180.0;

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

  inline Vector3d ToEulerAngle(const Matrix3d mat) {
    return Vector3d(
        atan2(mat(2, 1), mat(2, 2)),
        atan2(-mat(2, 0), sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2))),
        atan2(mat(1, 0), mat(0, 0)));
  }
};

}  // namespace INS

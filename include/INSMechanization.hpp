/**
 * @file INSMechanization.hpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief INS mechanization class
 * @version 1.0
 * @date 2021-06-18
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#pragma once

#include <Eigen/Geometry>

#include "DataStorage.hpp"

using namespace Eigen;

namespace INS {

class INSMechanization {
public:
  INSMechanization(double init_time, Vector3d init_theta,
                   Vector3d init_velocity, double init_phi, double init_lamda,
                   double init_height, Vector3d init_delta_theta,
                   Vector3d init_delta_velocity);
  ~INSMechanization();

  void SensorUpdate(double time, Vector3d delta_theta, Vector3d delta_velocity);
  INSStorage MechanizationUpdate();

private:
  // IMU output
  Vector3d delta_theta_, last_delta_theta_;
  Vector3d delta_v_, last_delta_v_;

  // INS mechanization data
  double time_, last_time_, delta_time_;
  Vector3d v_proj_n_, last_v_proj_n_, before_last_v_proj_n_;
  Quaterniond q_n_to_e_, q_n_last_to_e_last_;
  double h_, h_last_;
  Quaterniond q_b_to_n_, q_b_last_to_n_last_;

  Vector3d omega_ie_midway_proj_n_;
  Vector3d omega_en_midway_proj_n_;

  Vector3d v_midway_proj_n_;
  Quaterniond q_n_midway_to_e_midway_;
  double h_midway_;
  Vector3d g_proj_n_;

  // constant
  const double a_ = 6378137.0;
  const double e_2_ = 0.00669437999013;
  const double omega_e_ = 7.2921151467e-5;
  const Vector3d omega_ie_proj_e_;
  const double pi_ = 3.1415926535897932384626433832795;
  const double rad2degree_ = 57.29577951308232;
  const double degree2rad_ = 0.017453292519943295;

  // member function
  void VelocityUpdate();
  void PositionUpdate();
  void AttitudeUpdate();
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

  inline Vector2d ToGeodeticVector(const Matrix3d mat) {
    return Vector2d(atan2(-mat(2, 2), mat(2, 0)), atan2(-mat(0, 1), mat(1, 1)));
  }

  inline AngleAxisd ToAngleAxis(Vector3d vec) {
    return AngleAxisd(vec.norm(), vec.normalized());
  }

  Vector3d CalaulateZeta(const Vector3d v_proj_n, const Quaterniond q_n_to_e,
                         const double h, const double delta_t) {
    Vector2d geodetic_vec = ToGeodeticVector(q_n_to_e.toRotationMatrix());

    double R_M =
        a_ * (1 - e_2_) / pow(1 - e_2_ * pow(sin(geodetic_vec(0)), 2), 1.5);
    double R_N = a_ / sqrt(1 - e_2_ * pow(sin(geodetic_vec(0)), 2));

    omega_ie_midway_proj_n_ = Vector3d(omega_e_ * cos(geodetic_vec(0)), 0,
                                       -omega_e_ * sin(geodetic_vec(0)));

    omega_en_midway_proj_n_ =
        Vector3d(v_proj_n(1) / (R_N + h), -v_proj_n(0) / (R_M + h),
                 -v_proj_n(1) * tan(geodetic_vec(0)) / (R_N + h));

    return (omega_ie_midway_proj_n_ + omega_en_midway_proj_n_) * delta_t;
  }
};

}  // namespace INS

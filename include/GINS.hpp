/**
 * @file GINS.hpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief GINS class
 * @version 1.0
 * @date 2021-07-14
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#pragma once

#include <Eigen/Geometry>
#include <iostream>

#include "DataStorage.hpp"
#include "INSMechanization.hpp"
#include "Utils.hpp"

namespace iNav {

using ErrorType = Eigen::Matrix<double, 21, 1>;
using qType = Eigen::Matrix<double, 18, 18>;
using GType = Eigen::Matrix<double, 21, 18>;
using HType = Eigen::Matrix<double, 3, 21>;
using KType = Eigen::Matrix<double, 21, 3>;
using Matrix21d = Eigen::Matrix<double, 21, 21>;

class GINS : INS::INSMechanization {
public:
  GINS(const iNav::NavData& init_nav_data, const IMUParam& init_imu_param,
       const Eigen::Vector3d& l_b, const iNav::IMUData& init_imu_data);
  ~GINS();

  NavData Mechanization(IMUData data, bool record_data = true);
  void Prediction(bool record_data = true);
  NavData GNSSUpdate(GnssData gnss_data, IMUData imu_data);

private:
  Eigen::Vector3d gyro_bias_, acc_bias_, gyro_scalar_, acc_scalar_;
  Eigen::Vector3d l_b_;
  ErrorType delta_x_;
  qType q_;
  GType G_, last_G_;
  Matrix21d P_, Phi_, Q_;
  IMUParam IMU_param_;

  inline Eigen::Vector3d CorrectGyroError(const Eigen::Vector3d& gyro_raw) {
    return Eigen::Vector3d((gyro_raw - gyro_bias_ * delta_time_).array() /
                           (1 + gyro_scalar_.array()));
  }
  inline Eigen::Vector3d CorrectAccError(const Eigen::Vector3d& acc_raw) {
    return Eigen::Vector3d((acc_raw - acc_bias_ * delta_time_).array() /
                           (1 + acc_scalar_.array()));
  }

  Matrix21d SetP(const Eigen::Vector3d& position_std,
                 const Eigen::Vector3d& velocity_std,
                 const Eigen::Vector3d& theta_std, const IMUParam& param);
  qType Setq(const IMUParam& param);
  Matrix21d ComputeF();
  GType ComputeG(const Eigen::Quaterniond& q_b_to_n);
  HType ComputeHr();
  NavData NavDataInterpolation(const NavData& last_nav_data,
                               const NavData& nav_data, const double& ratio,
                               const double& timestamp);
  NavData CorrectNavDataAndIMUError(const NavData& data, ErrorType& error);
  Eigen::Vector3d ComputeZr(const GnssData& gnss_data, const NavData& nav_data);
};

}  // namespace iNav

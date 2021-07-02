#pragma once

#include <Eigen/Geometry>

#include "DataStorage.hpp"
#include "INSMechanization.hpp"
#include "Utils.hpp"

namespace iNav {

using ErrorType = Eigen::Matrix<double, 21, 1>;
using NoiseType = Eigen::Matrix<double, 18, 1>;
using qType = Eigen::Matrix<double, 18, 18>;
using GType = Eigen::Matrix<double, 21, 18>;
using Matrix21d = Eigen::Matrix<double, 21, 21>;

class GINS {
public:
  GINS(double init_time, Eigen::Vector3d init_position,
       Eigen::Vector3d init_position_std, Eigen::Vector3d init_velocity,
       Eigen::Vector3d init_velocity_std, Eigen::Vector3d init_theta,
       Eigen::Vector3d init_theta_std, IMUParam init_param,
       Eigen::Vector3d init_delta_theta, Eigen::Vector3d init_delta_velocity);
  ~GINS();

  NavData MechanizationUpdate(IMUData data);
  void Prediction();

private:
  double time_, last_time_, delta_time_;
  Eigen::Vector3d delta_r_, delta_v_, Phi_;
  Eigen::Vector3d gyro_bias_, acc_bias_, gyro_scalar_, acc_scalar_;
  ErrorType delta_x_, last_delta_x_;
  NoiseType w_, last_w_;
  qType q_;
  GType G_, last_G_, Q_, last_Q_;
  Matrix21d P_, last_P_;
  INS::INSMechanization ins_mechanization_;

  Matrix21d SetP(Eigen::Vector3d position_std, Eigen::Vector3d velocity_std,
                 Eigen::Vector3d theta_std, IMUParam param);

  qType Setq(IMUParam param);

  inline Eigen::Vector3d CorrectGyroError(Eigen::Vector3d gyro_raw) {
    return Eigen::Vector3d((gyro_raw - gyro_bias_ * delta_time_).array() /
                           (1 + gyro_scalar_.array()));
  }
  inline Eigen::Vector3d CorrectAccError(Eigen::Vector3d acc_raw) {
    return Eigen::Vector3d((acc_raw - acc_bias_ * delta_time_).array() /
                           (1 + acc_scalar_.array()));
  }
};

}  // namespace iNav

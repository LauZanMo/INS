#include "GINS.hpp"

namespace iNav {

GINS::GINS(double init_time, Eigen::Vector3d init_position,
           Eigen::Vector3d init_position_std, Eigen::Vector3d init_velocity,
           Eigen::Vector3d init_velocity_std, Eigen::Vector3d init_theta,
           Eigen::Vector3d init_theta_std, IMUParam init_param,
           Eigen::Vector3d init_delta_theta,
           Eigen::Vector3d init_delta_velocity)
    : last_time_(init_time),
      ins_mechanization_(init_time, init_position, init_velocity, init_theta,
                         init_delta_theta, init_delta_velocity),
      delta_r_(0, 0, 0),
      delta_v_(0, 0, 0),
      Phi_(0, 0, 0),
      gyro_bias_(0, 0, 0),
      acc_bias_(0, 0, 0),
      gyro_scalar_(0, 0, 0),
      acc_scalar_(0, 0, 0) {
  last_delta_x_ << delta_r_, delta_v_, Phi_, gyro_bias_, acc_bias_,
      gyro_scalar_, acc_scalar_;

  IMUParam param;
  param.VRW = init_param.VRW * DEGREE_PER_SQRT_HOUR_2_RAD_PER_SQRT_SECOND;
  param.ARW = init_param.ARW *
              METER_PER_SECOND_SQRT_HOUR_2_METER_PER_SECOND_SQRT_SECOND;

  param.gyro_bias_std =
      init_param.gyro_bias_std * DEGREE_PER_HOUR_2_RAD_PER_SECOND;
  param.acc_bias_std = init_param.acc_bias_std * MGAL_2_METER_PER_SECOND_SQUARE;
  param.T_gyro_bias = init_param.T_gyro_bias * HOUR_2_SECOND;
  param.T_acc_bias = init_param.T_acc_bias * HOUR_2_SECOND;

  param.gyro_scalar_std = init_param.gyro_scalar_std * PPM_2_1;
  param.acc_scalar_std = init_param.acc_scalar_std * PPM_2_1;
  param.T_gyro_scalar = init_param.T_gyro_scalar * HOUR_2_SECOND;
  param.T_acc_scalar = init_param.T_acc_scalar * HOUR_2_SECOND;

  last_P_ = SetP(init_position_std, init_velocity_std, init_theta_std, param);

  q_ = Setq(param);
}

GINS::~GINS() {}

NavData GINS::MechanizationUpdate(IMUData data) {
  time_ = data.timestamp;
  delta_time_ = data.timestamp - last_time_;

  IMUData correct_data;
  correct_data.timestamp = data.timestamp;
  correct_data.gyro = CorrectGyroError(data.gyro);
  correct_data.acc = CorrectAccError(data.acc);

  ins_mechanization_.SensorUpdate(correct_data);
  NavData output = ins_mechanization_.MechanizationUpdate();

  return output;
}

void GINS::Prediction() {}

Matrix21d GINS::SetP(Eigen::Vector3d position_std, Eigen::Vector3d velocity_std,
                     Eigen::Vector3d theta_std, IMUParam param) {
  Matrix21d mat = Matrix21d::Zero();

  mat.block(0, 0, 3, 3) =
      Eigen::Vector3d(position_std.array().pow(2)).asDiagonal();

  mat.block(3, 3, 3, 3) =
      Eigen::Vector3d(velocity_std.array().pow(2)).asDiagonal();

  mat.block(6, 6, 3, 3) =
      Eigen::Vector3d(theta_std.array().pow(2)).asDiagonal();

  mat.block(9, 9, 3, 3) =
      pow(param.gyro_bias_std, 2) * Eigen::Matrix3d::Identity();
  mat.block(12, 12, 3, 3) =
      pow(param.acc_bias_std, 2) * Eigen::Matrix3d::Identity();
  mat.block(15, 15, 3, 3) =
      pow(param.gyro_scalar_std, 2) * Eigen::Matrix3d::Identity();
  mat.block(18, 18, 3, 3) =
      pow(param.acc_scalar_std, 2) * Eigen::Matrix3d::Identity();

  return mat;
}

qType GINS::Setq(IMUParam param) {
  qType mat = qType::Zero();

  mat.block(0, 0, 3, 3) = pow(param.VRW, 2) * Eigen::Matrix3d::Identity();
  mat.block(3, 3, 3, 3) = pow(param.ARW, 2) * Eigen::Matrix3d::Identity();

  mat.block(6, 6, 3, 3) = 2 * pow(param.gyro_bias_std, 2) / param.T_gyro_bias *
                          Eigen::Matrix3d::Identity();
  mat.block(9, 9, 3, 3) = 2 * pow(param.acc_bias_std, 2) / param.T_acc_bias *
                          Eigen::Matrix3d::Identity();
  mat.block(12, 12, 3, 3) = 2 * pow(param.gyro_scalar_std, 2) /
                            param.T_gyro_scalar * Eigen::Matrix3d::Identity();
  mat.block(15, 15, 3, 3) = 2 * pow(param.acc_scalar_std, 2) /
                            param.T_acc_scalar * Eigen::Matrix3d::Identity();

  return mat;
}

}  // namespace iNav

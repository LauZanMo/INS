/**
 * @file GINS.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief GINS class
 * @version 1.0
 * @date 2021-07-14
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include "GINS.hpp"

// comment to use raw data interpolation
#define POSE_INTERPOLATION

namespace iNav {

GINS::GINS(const iNav::NavData& init_nav_data, const IMUParam& init_imu_param,
           const Eigen::Vector3d& l_b, const iNav::IMUData& init_imu_data)
    : INS::INSMechanization(init_nav_data, init_imu_data),
      l_b_(l_b),
      gyro_bias_(0, 0, 0),
      acc_bias_(0, 0, 0),
      gyro_scalar_(0, 0, 0),
      acc_scalar_(0, 0, 0) {
  delta_x_ = ErrorType::Zero();

  IMU_param_.ARW =
      init_imu_param.ARW * DEGREE_PER_SQRT_HOUR_2_RAD_PER_SQRT_SECOND;
  IMU_param_.VRW = init_imu_param.VRW *
                   METER_PER_SECOND_SQRT_HOUR_2_METER_PER_SECOND_SQRT_SECOND;

  IMU_param_.gyro_bias_std =
      init_imu_param.gyro_bias_std * DEGREE_PER_HOUR_2_RAD_PER_SECOND;
  IMU_param_.acc_bias_std =
      init_imu_param.acc_bias_std * MGAL_2_METER_PER_SECOND_SQUARE;
  IMU_param_.T_gyro_bias = init_imu_param.T_gyro_bias * HOUR_2_SECOND;
  IMU_param_.T_acc_bias = init_imu_param.T_acc_bias * HOUR_2_SECOND;

  IMU_param_.gyro_scalar_std = init_imu_param.gyro_scalar_std * PPM_2_1;
  IMU_param_.acc_scalar_std = init_imu_param.acc_scalar_std * PPM_2_1;
  IMU_param_.T_gyro_scalar = init_imu_param.T_gyro_scalar * HOUR_2_SECOND;
  IMU_param_.T_acc_scalar = init_imu_param.T_acc_scalar * HOUR_2_SECOND;

  P_ = SetP(init_nav_data.pos_std, init_nav_data.vel_std, init_nav_data.att_std,
            IMU_param_);

  q_ = Setq(IMU_param_);

  last_G_ = ComputeG(q_b_last_to_n_last_);
}

GINS::~GINS() {}

NavData GINS::Mechanization(IMUData data, bool record_data) {
  IMUData correct_data;
  correct_data.timestamp = data.timestamp;
  correct_data.gyro = CorrectGyroError(data.gyro);
  correct_data.acc = CorrectAccError(data.acc);

  SensorUpdate(correct_data);
  NavData output = MechanizationUpdate(record_data);

  return output;
}

void GINS::Prediction(bool record_data) {
  Matrix21d F = ComputeF();
  G_ = ComputeG(q_b_to_n_);
  Phi_ = Matrix21d::Identity() + F * delta_time_;
  Q_ = 0.5 *
       (Phi_ * last_G_ * q_ * last_G_.transpose() * Phi_.transpose() +
        G_ * q_ * G_.transpose()) *
       delta_time_;

  // prediction
  delta_x_ = Phi_ * delta_x_;
  P_ = Phi_ * P_ * Phi_.transpose() + Q_;

  // data record
  if (record_data) last_G_ = G_;
}

NavData GINS::GNSSUpdate(GnssData gnss_data, IMUData imu_data) {
  NavData correct_nav_data;

  if (gnss_data.timestamp == imu_data.timestamp) {
    // mechanization and prediction
    NavData nav_data = Mechanization(imu_data);
    Prediction();

    // gnss update
    Eigen::Matrix3d R =
        Eigen::Vector3d(gnss_data.pos_std.array().pow(2)).asDiagonal();
    HType H_r = ComputeHr();
    KType K = P_ * H_r.transpose() * (H_r * P_ * H_r.transpose() + R).inverse();
    Eigen::Vector3d z_r = ComputeZr(gnss_data, nav_data);
    delta_x_ = delta_x_ + K * (z_r - H_r * delta_x_);
    P_ = (Matrix21d::Identity() - K * H_r) * P_ *
             (Matrix21d::Identity() - K * H_r).transpose() +
         K * R * K.transpose();

    // correct nav data and IMU error
    correct_nav_data = CorrectNavDataAndIMUError(nav_data, delta_x_);
  } else if (gnss_data.timestamp < imu_data.timestamp) {
#ifndef POSE_INTERPOLATION
    // interpolation
    double ratio1 =
        (gnss_data.timestamp - last_time_) / (imu_data.timestamp - last_time_);
    double ratio2 = (imu_data.timestamp - gnss_data.timestamp) /
                    (imu_data.timestamp - last_time_);
    IMUData imu_gps_virtual;
    imu_gps_virtual.timestamp = gnss_data.timestamp;
    imu_gps_virtual.gyro = ratio1 * imu_data.gyro;
    imu_gps_virtual.acc = ratio1 * imu_data.acc;

    IMUData imu_virtual;
    imu_virtual.timestamp = imu_data.timestamp;
    imu_virtual.gyro = ratio2 * imu_data.gyro;
    imu_virtual.acc = ratio2 * imu_data.acc;

    // mechanization and prediction
    NavData nav_data = Mechanization(imu_gps_virtual);
    Prediction();

    // gnss update
    Eigen::Matrix3d R =
        Eigen::Vector3d(gnss_data.pos_std.array().pow(2)).asDiagonal();
    HType H_r = ComputeHr();
    KType K = P_ * H_r.transpose() * (H_r * P_ * H_r.transpose() + R).inverse();
    Eigen::Vector3d z_r = ComputeZr(gnss_data, nav_data);
    delta_x_ = delta_x_ + K * (z_r - H_r * delta_x_);
    P_ = (Matrix21d::Identity() - K * H_r) * P_ *
             (Matrix21d::Identity() - K * H_r).transpose() +
         K * R * K.transpose();

    // correct nav data and IMU error
    CorrectNavDataAndIMUError(nav_data, delta_x_);

    // mechanization and prediction
    correct_nav_data = Mechanization(imu_virtual);
    Prediction();
#else
    double ratio =
        (gnss_data.timestamp - last_time_) / (imu_data.timestamp - last_time_);

    // mechanization
    NavData last_nav_data = GetNavState();
    NavData nav_data = Mechanization(imu_data, false);

    // interpolation
    NavData gnss_interpolation_data = NavDataInterpolation(
        last_nav_data, nav_data, ratio, gnss_data.timestamp);

    // prediction
    SetNavState(gnss_interpolation_data);
    Prediction(false);

    // gnss update
    Eigen::Matrix3d R =
        Eigen::Vector3d(gnss_data.pos_std.array().pow(2)).asDiagonal();
    HType H_r = ComputeHr();
    KType K = P_ * H_r.transpose() * (H_r * P_ * H_r.transpose() + R).inverse();
    Eigen::Vector3d z_r = ComputeZr(gnss_data, gnss_interpolation_data);
    delta_x_ = delta_x_ + K * (z_r - H_r * delta_x_);
    P_ = (Matrix21d::Identity() - K * H_r) * P_ *
             (Matrix21d::Identity() - K * H_r).transpose() +
         K * R * K.transpose();

    // inverse
    delta_x_ = Phi_.inverse() * delta_x_;
    P_ = Phi_.inverse() * (P_ - Q_) * Phi_.transpose().inverse();

    // correct nav data and IMU error
    SetNavState(last_nav_data);
    CorrectNavDataAndIMUError(last_nav_data, delta_x_);

    // mechanization and prediction
    correct_nav_data = Mechanization(imu_data);
    Prediction();
#endif
  }

  return correct_nav_data;
}

Matrix21d GINS::SetP(const Eigen::Vector3d& position_std,
                     const Eigen::Vector3d& velocity_std,
                     const Eigen::Vector3d& theta_std, const IMUParam& param) {
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

qType GINS::Setq(const IMUParam& param) {
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

Matrix21d GINS::ComputeF() {
  Eigen::Vector2d geodetic_vec = Quat2GeodeticVec(q_n_to_e_);
  double lat = geodetic_vec(0);
  Eigen::Vector2d R = ComputeRmRn(lat);
  Eigen::Vector3d g_n = ComputeGn(lat, h_);

  // compute F_rr
  Eigen::Matrix3d F_rr = Eigen::Matrix3d::Zero();
  F_rr(0, 0) = -v_proj_n_(2) / (R(0) + h_);
  F_rr(0, 2) = v_proj_n_(0) / (R(0) + h_);
  F_rr(1, 0) = v_proj_n_(1) * tan(lat) / (R(1) + h_);
  F_rr(1, 1) = -(v_proj_n_(2) + v_proj_n_(0) * tan(lat)) / (R(1) + h_);
  F_rr(1, 2) = v_proj_n_(1) / (R(1) + h_);

  // compute F_vr
  Eigen::Matrix3d F_vr = Eigen::Matrix3d::Zero();
  F_vr(0, 0) = -2 * v_proj_n_(1) * OMEGA_E * cos(lat) / (R(0) + h_) -
               pow(v_proj_n_(1) / cos(lat), 2) / ((R(0) + h_) * (R(1) + h_));
  F_vr(0, 2) = v_proj_n_(0) * v_proj_n_(2) / pow(R(0) + h_, 2) -
               pow(v_proj_n_(1) / (R(1) + h_), 2) * tan(lat);
  F_vr(1, 0) = 2 * OMEGA_E *
                   (v_proj_n_(0) * cos(lat) - v_proj_n_(2) * sin(lat)) /
                   (R(0) + h_) +
               v_proj_n_(0) * v_proj_n_(1) * pow(1 / cos(lat), 2) /
                   ((R(0) + h_) * (R(1) + h_));
  F_vr(1, 2) =
      (v_proj_n_(1) * v_proj_n_(2) + v_proj_n_(0) * v_proj_n_(1) * tan(lat)) /
      pow(R(1) + h_, 2);
  F_vr(2, 0) = 2 * OMEGA_E * v_proj_n_(1) * sin(lat) / (R(0) + h_);
  F_vr(2, 2) = -pow(v_proj_n_(1) / (R(1) + h_), 2) -
               pow(v_proj_n_(0) / (R(0) + h_), 2) +
               2 * g_n(2) / (sqrt(R(0) * R(1)) + h_);

  // compute F_vv
  Eigen::Matrix3d F_vv = Eigen::Matrix3d::Zero();
  F_vv(0, 0) = v_proj_n_(2) / (R(0) + h_);
  F_vv(0, 1) =
      -2 * (OMEGA_E * sin(lat) + v_proj_n_(1) * tan(lat) / (R(1) + h_));
  F_vv(0, 2) = v_proj_n_(0) / (R(0) + h_);
  F_vv(1, 0) = 2 * OMEGA_E * sin(lat) + v_proj_n_(1) * tan(lat) / (R(1) + h_);
  F_vv(1, 1) = (v_proj_n_(2) + v_proj_n_(0) * tan(lat)) / (R(1) + h_);
  F_vv(1, 2) = 2 * OMEGA_E * cos(lat) + v_proj_n_(1) / (R(1) + h_);
  F_vv(2, 0) = -2 * v_proj_n_(0) / (R(0) + h_);
  F_vv(2, 1) = -2 * (OMEGA_E * cos(lat) + v_proj_n_(1) / (R(1) + h_));

  // compute F_phi_r
  Eigen::Matrix3d F_phi_r = Eigen::Matrix3d::Zero();
  F_phi_r(0, 0) = -OMEGA_E * sin(lat) / (R(0) + h_);
  F_phi_r(0, 2) = v_proj_n_(1) / pow(R(1) + h_, 2);
  F_phi_r(1, 2) = -v_proj_n_(0) / pow(R(0) + h_, 2);
  F_phi_r(2, 0) =
      -OMEGA_E * cos(lat) / (R(0) + h_) -
      v_proj_n_(1) * pow(1 / cos(lat), 2) / ((R(0) + h_) * (R(1) + h_));
  F_phi_r(2, 2) = -v_proj_n_(1) * tan(lat) / pow(R(1) + h_, 2);

  // compute F_phi_v
  Eigen::Matrix3d F_phi_v = Eigen::Matrix3d::Zero();
  F_phi_v(0, 1) = 1 / (R(1) + h_);
  F_phi_v(1, 0) = -1 / (R(0) + h_);
  F_phi_v(2, 1) = -tan(lat) / (R(1) + h_);

  // compute omega_in_proj_n
  Eigen::Vector3d omega_in_proj_n =
      ComputeOmegaIEProjN(lat) +
      ComputeOmegaENProjN(v_proj_n_, lat, h_, R(0), R(1));

  // compute omega_ib_proj_b and f_proj_b
  Eigen::Vector3d omega_ib_proj_b = delta_theta_ / delta_time_;
  Eigen::Vector3d f_proj_b = delta_v_ / delta_time_;

  // compute F
  Matrix21d F = Matrix21d::Zero();
  F.block(0, 0, 3, 3) = F_rr;
  F.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
  F.block(3, 0, 3, 3) = F_vr;
  F.block(3, 3, 3, 3) = F_vv;
  F.block(3, 6, 3, 3) =
      GetSkewSymmetricMat(q_b_to_n_.toRotationMatrix() * f_proj_b);
  F.block(3, 12, 3, 3) = q_b_to_n_.toRotationMatrix();
  F.block(3, 18, 3, 3) = q_b_to_n_.toRotationMatrix() * f_proj_b.asDiagonal();
  F.block(6, 0, 3, 3) = F_phi_r;
  F.block(6, 3, 3, 3) = F_phi_v;
  F.block(6, 6, 3, 3) = -GetSkewSymmetricMat(omega_in_proj_n);
  F.block(6, 9, 3, 3) = -q_b_to_n_.toRotationMatrix();
  F.block(6, 15, 3, 3) =
      -q_b_to_n_.toRotationMatrix() * omega_ib_proj_b.asDiagonal();
  F.block(9, 9, 3, 3) =
      -1 / IMU_param_.T_gyro_bias * Eigen::Matrix3d::Identity();
  F.block(12, 12, 3, 3) =
      -1 / IMU_param_.T_acc_bias * Eigen::Matrix3d::Identity();
  F.block(15, 15, 3, 3) =
      -1 / IMU_param_.T_gyro_scalar * Eigen::Matrix3d::Identity();
  F.block(18, 18, 3, 3) =
      -1 / IMU_param_.T_acc_scalar * Eigen::Matrix3d::Identity();
  return F;
}

GType GINS::ComputeG(const Eigen::Quaterniond& q_b_to_n) {
  GType G = GType::Zero();
  G.block(3, 0, 3, 3) = q_b_to_n.toRotationMatrix();
  G.block(6, 3, 3, 3) = q_b_to_n.toRotationMatrix();
  G.block(9, 6, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(12, 9, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(15, 12, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(18, 15, 3, 3) = Eigen::Matrix3d::Identity();
  return G;
}

HType GINS::ComputeHr() {
  HType H_r = HType::Zero();
  H_r.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  H_r.block(0, 6, 3, 3) =
      q_b_to_n_.toRotationMatrix() * GetSkewSymmetricMat(l_b_);
  return H_r;
}

NavData GINS::NavDataInterpolation(const NavData& last_nav_data,
                                   const NavData& nav_data, const double& ratio,
                                   const double& timestamp) {
  NavData output_data;
  output_data.timestamp = timestamp;
  output_data.pos =
      last_nav_data.pos + ratio * (last_nav_data.pos - nav_data.pos);
  output_data.vel =
      last_nav_data.vel + ratio * (last_nav_data.vel - nav_data.vel);

  Eigen::Quaterniond last_q_b_to_n =
      EulerAngle2Quat(DEGREE_2_RAD * last_nav_data.att);
  Eigen::Quaterniond q_b_to_n =
      EulerAngle2Quat(DEGREE_2_RAD * last_nav_data.att);

  Eigen::Quaterniond q_delta_theta = last_q_b_to_n.inverse() * q_b_to_n;
  Eigen::AngleAxisd vec(q_delta_theta);
  vec.angle() = ratio * vec.angle();
  Eigen::Quaterniond q_ratio_delta_theta(vec);
  output_data.att =
      RAD_2_DEGREE *
      DCM2EulerAngle((last_q_b_to_n * q_ratio_delta_theta).toRotationMatrix());

  return output_data;
}

Eigen::Vector3d GINS::ComputeZr(const GnssData& gnss_data,
                                const NavData& nav_data) {
  Eigen::Vector2d geodetic_vec = Quat2GeodeticVec(q_n_to_e_);
  double lat = geodetic_vec(0);
  Eigen::Vector2d R = ComputeRmRn(lat);

  Eigen::Matrix3d D_r =
      Eigen::Vector3d(R(0) + h_, (R(1) + h_) * cos(lat), -1).asDiagonal();

  Eigen::Vector3d delta_r_n = nav_data.pos - gnss_data.pos;
  delta_r_n.block(0, 0, 2, 1) *= DEGREE_2_RAD;

  Eigen::Vector3d Z_r = D_r * delta_r_n + q_b_to_n_.toRotationMatrix() * l_b_;
  return Z_r;
}

NavData GINS::CorrectNavDataAndIMUError(const NavData& data, ErrorType& error) {
  NavData correct_data;
  correct_data.timestamp = data.timestamp;
  // correct rotation vector error
  Eigen::Vector3d rotation_vec = error.block(6, 0, 3, 1);
  Eigen::Matrix3d C_b_to_p =
      EulerAngle2Quat(data.att * DEGREE_2_RAD).toRotationMatrix();
  Eigen::Matrix3d C_t_to_p =
      Eigen::Matrix3d::Identity() - GetSkewSymmetricMat(rotation_vec);
  q_b_to_n_ = C_t_to_p.transpose() * C_b_to_p;
  correct_data.att =
      RAD_2_DEGREE * DCM2EulerAngle(q_b_to_n_.toRotationMatrix());

  // correct position error
  Eigen::Vector3d delta_r = error.block(0, 0, 3, 1);
  Eigen::Vector2d geodetic_vec = Quat2GeodeticVec(q_n_to_e_);
  double lat = geodetic_vec(0);
  Eigen::Vector2d R = ComputeRmRn(lat);

  Eigen::Matrix3d D_r_inverse =
      Eigen::Vector3d(1 / (R(0) + h_), 1 / ((R(1) + h_) * cos(lat)), -1)
          .asDiagonal();
  Eigen::Vector3d r_BLH;
  r_BLH << geodetic_vec, h_;
  r_BLH -= D_r_inverse * delta_r;

  q_n_to_e_ = GeodeticVec2Quat(r_BLH.block(0, 0, 2, 1));
  h_ = r_BLH(2);
  r_BLH.block(0, 0, 2, 1) *= RAD_2_DEGREE;
  correct_data.pos = r_BLH;

  // correct velocity error
  Eigen::Vector3d delta_v = error.block(3, 0, 3, 1);
  v_proj_n_ = C_t_to_p.transpose() * v_proj_n_;
  v_proj_n_ -= delta_v;
  correct_data.vel = v_proj_n_;

  // correct bias and scalar
  Eigen::Vector3d delta_gyro_bias = error.block(9, 0, 3, 1);
  Eigen::Vector3d delta_acc_bias = error.block(12, 0, 3, 1);
  Eigen::Vector3d delta_gyro_scalar = error.block(15, 0, 3, 1);
  Eigen::Vector3d delta_acc_scalar = error.block(18, 0, 3, 1);

  gyro_bias_ = gyro_bias_ + Eigen::Vector3d((1 + gyro_scalar_.array()) *
                                            delta_gyro_bias.array());
  gyro_scalar_ = Eigen::Vector3d(
      (1 + gyro_scalar_.array()) * (1 + delta_gyro_scalar.array()) - 1);

  acc_bias_ = acc_bias_ + Eigen::Vector3d((1 + acc_scalar_.array()) *
                                          delta_acc_bias.array());
  acc_scalar_ = Eigen::Vector3d(
      (1 + acc_scalar_.array()) * (1 + delta_acc_scalar.array()) - 1);

  // reset error and record correct data
  error.setZero();
  last_v_proj_n_ = v_proj_n_;
  q_n_last_to_e_last_ = q_n_to_e_;
  h_last_ = h_;
  q_b_last_to_n_last_ = q_b_to_n_;

  return correct_data;
}

}  // namespace iNav

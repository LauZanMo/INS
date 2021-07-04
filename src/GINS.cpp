#include "GINS.hpp"

namespace iNav {

GINS::GINS(const double& init_time, const Eigen::Vector3d& init_position,
           const Eigen::Vector3d& init_position_std,
           const Eigen::Vector3d& init_velocity,
           const Eigen::Vector3d& init_velocity_std,
           const Eigen::Vector3d& init_theta,
           const Eigen::Vector3d& init_theta_std,
           const IMUParam& init_imu_param, const Eigen::Vector3d& l_b,
           const Eigen::Vector3d& init_delta_theta,
           const Eigen::Vector3d& init_delta_velocity)
    : INS::INSMechanization(init_time, init_position, init_velocity, init_theta,
                            init_delta_theta, init_delta_velocity),
      l_b_(l_b),
      gyro_bias_(0, 0, 0),
      acc_bias_(0, 0, 0),
      gyro_scalar_(0, 0, 0),
      acc_scalar_(0, 0, 0) {
  last_delta_x_ = ErrorType::Zero();

  IMU_param_.VRW =
      init_imu_param.VRW * DEGREE_PER_SQRT_HOUR_2_RAD_PER_SQRT_SECOND;
  IMU_param_.ARW = init_imu_param.ARW *
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

  last_P_ =
      SetP(init_position_std, init_velocity_std, init_theta_std, IMU_param_);

  q_ = Setq(IMU_param_);
}

GINS::~GINS() {}

NavData GINS::Mechanization(IMUData data) {
  IMUData correct_data;
  correct_data.timestamp = data.timestamp;
  correct_data.gyro = CorrectGyroError(data.gyro);
  correct_data.acc = CorrectAccError(data.acc);

  SensorUpdate(correct_data);
  NavData output = MechanizationUpdate();

  return output;
}

void GINS::Prediction() {
  Matrix21d F = ComputeF();
  G_ = ComputeG();
  Matrix21d Phi = Matrix21d::Identity() + F * delta_time_;
  Matrix21d Q = 0.5 *
                (Phi * last_G_ * q_ * last_G_.transpose() * Phi.transpose() +
                 G_ * q_ * G_.transpose()) *
                delta_time_;

  // prediction
  delta_x_ = Phi * last_delta_x_;
  P_ = Phi * last_P_ * Phi.transpose() + Q;

  // data record
  last_delta_x_ = delta_x_;
  last_G_ = G_;
  last_P_ = P_;
}

NavData GINS::GNSSUpdate(GnssData gnss_data, IMUData imu_data) {
  NavData correct_nav_data;
  GnssData trans_gnss_data = gnss_data;
  trans_gnss_data.pos.block(0, 0, 2, 1) *= DEGREE_2_RAD;

  if (trans_gnss_data.timestamp == imu_data.timestamp) {
    NavData nav_data = Mechanization(imu_data);
    Prediction();

    Eigen::Matrix3d R =
        Eigen::Vector3d(trans_gnss_data.pos_std.array().pow(2)).asDiagonal();
    HType H_r = ComputeHr();
    KType K = P_ * H_r.transpose() * (H_r * P_ * H_r.transpose() + R).inverse();
    delta_x_ = last_delta_x_ + K * (trans_gnss_data.pos - H_r * last_delta_x_);
    P_ = (Matrix21d::Identity() - K * H_r) * last_P_ *
             (Matrix21d::Identity() - K * H_r).transpose() +
         K * R * K.transpose();

    correct_nav_data = CorrectNavDataAndIMUError(nav_data, delta_x_);
  } else if (trans_gnss_data.timestamp < imu_data.timestamp) {
    IMUData imu_gps_virtual;
    imu_gps_virtual.timestamp = trans_gnss_data.timestamp;
    imu_gps_virtual.gyro = (trans_gnss_data.timestamp - last_time_) /
                           (imu_data.timestamp - last_time_) * imu_data.gyro;
    imu_gps_virtual.acc = (trans_gnss_data.timestamp - last_time_) /
                          (imu_data.timestamp - last_time_) * imu_data.acc;

    IMUData imu_virtual;
    imu_virtual.timestamp = imu_data.timestamp;
    imu_virtual.gyro = (imu_data.timestamp - trans_gnss_data.timestamp) /
                       (imu_data.timestamp - last_time_) * imu_data.gyro;
    imu_virtual.acc = (imu_data.timestamp - trans_gnss_data.timestamp) /
                      (imu_data.timestamp - last_time_) * imu_data.acc;

    NavData nav_data = Mechanization(imu_gps_virtual);
    Prediction();

    Eigen::Matrix3d R =
        Eigen::Vector3d(trans_gnss_data.pos_std.array().pow(2)).asDiagonal();
    HType H_r = ComputeHr();
    KType K = P_ * H_r.transpose() * (H_r * P_ * H_r.transpose() +
    R).inverse(); delta_x_ = last_delta_x_ + K * (trans_gnss_data.pos - H_r *
    last_delta_x_); P_ = (Matrix21d::Identity() - K * H_r) * last_P_ *
             (Matrix21d::Identity() - K * H_r).transpose() +
         K * R * K.transpose();

    CorrectNavDataAndIMUError(nav_data, delta_x_);

    correct_nav_data = Mechanization(imu_virtual);
    Prediction();
  }

  // data record
  last_delta_x_ = delta_x_;
  last_P_ = P_;

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

GType GINS::ComputeG() {
  GType G = GType::Zero();
  G.block(3, 0, 3, 3) = q_b_to_n_.toRotationMatrix();
  G.block(6, 3, 3, 3) = q_b_to_n_.toRotationMatrix();
  G.block(9, 6, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(12, 9, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(15, 12, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(18, 15, 3, 3) = Eigen::Matrix3d::Identity();
  return G;
}

HType GINS::ComputeHr() {
  HType H_r = HType::Zero();
  H_r.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  H_r.block(0, 0, 3, 3) =
      q_b_to_n_.toRotationMatrix() * GetSkewSymmetricMat(l_b_);
  return H_r;
}

NavData GINS::CorrectNavDataAndIMUError(const NavData& data, ErrorType& error) {
  NavData correct_data;
  correct_data.timestamp = data.timestamp;
  // correct attitude error
  Eigen::Vector3d delta_phi = error.block(6, 0, 3, 1);
  Eigen::Matrix3d C_b_to_p =
      EulerAngle2Quat(data.att * DEGREE_2_RAD).toRotationMatrix();
  Eigen::Matrix3d C_t_to_p =
      Eigen::Matrix3d::Identity() - GetSkewSymmetricMat(delta_phi);
  q_b_to_n_ = C_t_to_p.inverse() * C_b_to_p;
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
  v_proj_n_ = C_t_to_p.inverse() * v_proj_n_;
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

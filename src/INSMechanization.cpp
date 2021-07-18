/**
 * @file INSMechanization.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief INS mechanization class
 * @version 1.0
 * @date 2021-06-18
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include "INSMechanization.hpp"

#include <iostream>

using namespace std;

namespace INS {

/**
 * @brief Construct a new INSMechanization::INSMechanization object
 *
 * @param init_nav_data
 * @param init_imu_data
 */
INSMechanization::INSMechanization(const iNav::NavData& init_nav_data,
                                   const iNav::IMUData& init_imu_data)
    : last_time_(init_nav_data.timestamp),
      last_v_proj_n_(init_nav_data.vel),
      before_last_v_proj_n_(init_nav_data.vel),
      h_last_(init_nav_data.pos(2)),
      last_delta_theta_(init_imu_data.gyro),
      last_delta_v_(init_imu_data.acc) {
  q_n_last_to_e_last_ = iNav::GeodeticVec2Quat(
      iNav::DEGREE_2_RAD * init_nav_data.pos.block(0, 0, 2, 1));

  q_b_last_to_n_last_ =
      iNav::EulerAngle2Quat(iNav::DEGREE_2_RAD * init_nav_data.att);
}

/**
 * @brief destroy the INSMechanization::INS object
 */
INSMechanization::~INSMechanization() {}

/**
 * @brief sensor update
 *
 * @param data imu data
 */
void INSMechanization::SensorUpdate(const iNav::IMUData& data) {
  time_ = data.timestamp;
  delta_time_ = data.timestamp - last_time_;

  delta_theta_ = data.gyro;
  delta_v_ = data.acc;
}

/**
 * @brief mechanization update
 *
 * @param record_data determine whether to record data or not
 * @return iNav::NavData
 */
iNav::NavData INSMechanization::MechanizationUpdate(bool record_data) {
  VelocityUpdate();
  PositionUpdate();
  AttitudeUpdate();

  iNav::NavData mechanization_output = GetNavState();

  if (record_data) DataRecord();

  return mechanization_output;
}

/**
 * @brief get nav state
 *
 * @return iNav::NavData: nav state
 */
iNav::NavData INSMechanization::GetNavState() {
  iNav::NavData nav_state;
  nav_state.timestamp = time_;

  nav_state.att =
      iNav::RAD_2_DEGREE * iNav::DCM2EulerAngle(q_b_to_n_.toRotationMatrix());

  nav_state.vel = v_proj_n_;

  Eigen::Vector2d geodetic_vec =
      iNav::RAD_2_DEGREE * iNav::Quat2GeodeticVec(q_n_to_e_);
  nav_state.pos << geodetic_vec, h_;

  return nav_state;
}

/**
 * @brief set nav state and time stamp
 *
 * @param nav_data nav state
 */
void INSMechanization::SetNavState(const iNav::NavData& nav_data) {
  time_ = nav_data.timestamp;
  delta_time_ = nav_data.timestamp - last_time_;

  q_b_to_n_ = iNav::EulerAngle2Quat(iNav::DEGREE_2_RAD * nav_data.att);

  v_proj_n_ = nav_data.vel;

  q_n_to_e_ = iNav::GeodeticVec2Quat(iNav::DEGREE_2_RAD *
                                     nav_data.pos.block(0, 0, 2, 1));

  h_ = nav_data.pos(2);
}

/**
 * @brief set nav state,imu data and time stamp
 * 
 * @param nav_data nav state
 * @param imu_data imu data
 */
void INSMechanization::SetNavState(const iNav::NavData& nav_data,
                                   const iNav::IMUData imu_data) {
  time_ = nav_data.timestamp;
  delta_time_ = nav_data.timestamp - last_time_;

  delta_v_ = imu_data.acc;
  delta_theta_ = imu_data.gyro;

  q_b_to_n_ = iNav::EulerAngle2Quat(iNav::DEGREE_2_RAD * nav_data.att);

  v_proj_n_ = nav_data.vel;

  q_n_to_e_ = iNav::GeodeticVec2Quat(iNav::DEGREE_2_RAD *
                                     nav_data.pos.block(0, 0, 2, 1));

  h_ = nav_data.pos(2);
}

/**
 * @brief velocity update
 */
void INSMechanization::VelocityUpdate() {
  // update midway data
  v_midway_proj_n_ = 1.5 * last_v_proj_n_ - 0.5 * before_last_v_proj_n_;

  Eigen::Vector3d zeta_midway = ComputeZeta(last_v_proj_n_, q_n_last_to_e_last_,
                                            h_last_, 0.5 * delta_time_);
  Eigen::Quaterniond q_n_midway_to_n_last(
      iNav::RotationVec2AngleAxis(zeta_midway));

  Eigen::Vector3d xi_midway = iNav::OMEGA_IE_PROJ_E * 0.5 * delta_time_;
  Eigen::Quaterniond q_e_last_to_e_midway(
      iNav::RotationVec2AngleAxis(-xi_midway));

  q_n_midway_to_e_midway_ =
      q_e_last_to_e_midway * q_n_last_to_e_last_ * q_n_midway_to_n_last;

  h_midway_ = h_last_ - last_v_proj_n_(2) * 0.5 * delta_time_;

  // compute delta_v_f_proj_n
  Eigen::Vector3d zeta = ComputeZeta(
      v_midway_proj_n_, q_n_midway_to_e_midway_, h_midway_, delta_time_,
      omega_ie_midway_proj_n_, omega_en_midway_proj_n_);

  Eigen::Vector3d delta_v_f_proj_b_last =
      delta_v_ + 0.5 * delta_theta_.cross(delta_v_) +
      1.0 / 12.0 *
          (last_delta_theta_.cross(delta_v_) +
           last_delta_v_.cross(delta_theta_));

  Eigen::Vector3d delta_v_f_proj_n =
      (Eigen::Matrix3d::Identity() - 0.5 * iNav::GetSkewSymmetricMat(zeta)) *
      q_b_last_to_n_last_.toRotationMatrix() * delta_v_f_proj_b_last;

  // compute delta_v_g_and_coriolis_proj_n
  Eigen::Vector2d geodetic_vec =
      iNav::Quat2GeodeticVec(q_n_midway_to_e_midway_);
  g_proj_n_ = iNav::ComputeGn(geodetic_vec(0), h_midway_);

  Eigen::Vector3d delta_v_g_and_coriolis_proj_n =
      (g_proj_n_ - (2 * omega_ie_midway_proj_n_ + omega_en_midway_proj_n_)
                       .cross(v_midway_proj_n_)) *
      delta_time_;

  // compute velocity
  v_proj_n_ = last_v_proj_n_ + delta_v_f_proj_n + delta_v_g_and_coriolis_proj_n;
}

/**
 * @brief position update
 */
void INSMechanization::PositionUpdate() {
  // update midway data
  v_midway_proj_n_ = 0.5 * (last_v_proj_n_ + v_proj_n_);

  // compute height
  h_ = h_last_ - v_midway_proj_n_(2) * delta_time_;

  // compute phi
  h_midway_ = 0.5 * (h_last_ + h_);

  Eigen::Vector2d last_geodetic_vec(
      iNav::Quat2GeodeticVec(q_n_last_to_e_last_));

  Eigen::Vector2d R = iNav::ComputeRmRn(last_geodetic_vec(0));

  Eigen::Vector2d geodetic_vec;
  geodetic_vec(0) = last_geodetic_vec(0) +
                    v_midway_proj_n_(0) / (R(0) + h_midway_) * delta_time_;

  // compute lamda
  double lat_midway = 0.5 * (last_geodetic_vec(0) + geodetic_vec(0));

  R = iNav::ComputeRmRn(lat_midway);

  geodetic_vec(1) = last_geodetic_vec(1) +
                    v_midway_proj_n_(1) /
                        ((R(1) + h_midway_) * cos(lat_midway)) * delta_time_;

  q_n_to_e_ = iNav::GeodeticVec2Quat(geodetic_vec);
  q_n_to_e_.normalize();
}

/**
 * @brief attitude update
 */
void INSMechanization::AttitudeUpdate() {
  // update midway data
  Eigen::Quaterniond q_delta_theta = q_n_last_to_e_last_.inverse() * q_n_to_e_;
  Eigen::AngleAxisd vec(q_delta_theta);
  vec.angle() = 0.5 * vec.angle();
  Eigen::Quaterniond q_half_delta_theta(vec);
  q_n_midway_to_e_midway_ = q_n_last_to_e_last_ * q_half_delta_theta;

  // compute q_b_to_b_last
  Eigen::Vector3d phi =
      delta_theta_ + 1.0 / 12.0 * last_delta_theta_.cross(delta_theta_);

  Eigen::Quaterniond q_b_to_b_last(iNav::RotationVec2AngleAxis(phi));

  // compute q_n_last_to_n
  Eigen::Vector3d zeta = ComputeZeta(v_midway_proj_n_, q_n_midway_to_e_midway_,
                                     h_midway_, delta_time_);
  Eigen::Quaterniond q_n_last_to_n(iNav::RotationVec2AngleAxis(-zeta));

  // compute theta
  q_b_to_n_ = q_n_last_to_n * q_b_last_to_n_last_ * q_b_to_b_last;
  q_b_to_n_.normalize();
}

/**
 * @brief record data
 */
void INSMechanization::DataRecord() {
  // time record
  last_time_ = time_;

  // IMU data record
  last_delta_theta_ = delta_theta_;
  last_delta_v_ = delta_v_;

  // INS data record
  before_last_v_proj_n_ = last_v_proj_n_;
  last_v_proj_n_ = v_proj_n_;
  q_n_last_to_e_last_ = q_n_to_e_;
  h_last_ = h_;
  q_b_last_to_n_last_ = q_b_to_n_;
}

/**
 * @brief compute zeta
 *
 * @param v_proj_n velocity projected on n-frame
 * @param q_n_to_e position
 * @param h height
 * @param delta_t delta time
 * @return Eigen::Vector3d: zeta value
 */
Eigen::Vector3d INSMechanization::ComputeZeta(
    const Eigen::Vector3d& v_proj_n, const Eigen::Quaterniond& q_n_to_e,
    const double& h, const double& delta_t) {
  Eigen::Vector2d geodetic_vec = iNav::Quat2GeodeticVec(q_n_to_e);

  Eigen::Vector2d R = iNav::ComputeRmRn(geodetic_vec(0));

  Eigen::Vector3d omega_ie_proj_n = iNav::ComputeOmegaIEProjN(geodetic_vec(0));
  Eigen::Vector3d omega_en_proj_n =
      iNav::ComputeOmegaENProjN(v_proj_n, geodetic_vec(0), h, R(0), R(1));

  return (omega_ie_proj_n + omega_en_proj_n) * delta_t;
}

/**
 * @brief compute zeta
 *
 * @param v_proj_n velocity projected on n-frame
 * @param q_n_to_e position
 * @param h height
 * @param delta_t delta time
 * @param omega_ie_proj_n angular rate from e-frame to i-frame projected on
 * n-frame (refer this variant and set value)
 * @param omega_en_proj_n angular rate from e-frame to n-frame projected on
 * n-frame (refer this variant and set value)
 * @return Eigen::Vector3d: zeta value
 */
Eigen::Vector3d INSMechanization::ComputeZeta(
    const Eigen::Vector3d& v_proj_n, const Eigen::Quaterniond& q_n_to_e,
    const double& h, const double& delta_t, Eigen::Vector3d& omega_ie_proj_n,
    Eigen::Vector3d& omega_en_proj_n) {
  Eigen::Vector2d geodetic_vec = iNav::Quat2GeodeticVec(q_n_to_e);

  Eigen::Vector2d R = iNav::ComputeRmRn(geodetic_vec(0));

  omega_ie_proj_n = iNav::ComputeOmegaIEProjN(geodetic_vec(0));
  omega_en_proj_n =
      iNav::ComputeOmegaENProjN(v_proj_n, geodetic_vec(0), h, R(0), R(1));

  return (omega_ie_proj_n + omega_en_proj_n) * delta_t;
}

}  // namespace INS

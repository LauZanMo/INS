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
 * @brief construct a new INSMechanization::INSMechanization object
 *
 * @param init_time INS init time(s)
 * @param init_theta INS init euler angle(deg)
 * @param init_velocity INS init velocity(m/s)
 * @param init_phi INS init latitude(deg)
 * @param init_lamda INS init longitude(deg)
 * @param init_height INS init altitude(m)
 * @param init_delta_theta INS init gyro data
 * @param init_delta_velocity INS init acc data
 */
INSMechanization::INSMechanization(double init_time,
                                   Eigen::Vector3d init_position,
                                   Eigen::Vector3d init_velocity,
                                   Eigen::Vector3d init_theta,
                                   Eigen::Vector3d init_delta_theta,
                                   Eigen::Vector3d init_delta_velocity)
    : last_time_(init_time),
      last_v_proj_n_(init_velocity),
      before_last_v_proj_n_(init_velocity),
      h_last_(init_position(2)),
      last_delta_theta_(init_delta_theta),
      last_delta_v_(init_delta_velocity) {
  q_n_last_to_e_last_ = iNav::GeodeticVec2Quat(
      Eigen::Vector2d(iNav::DEGREE_2_RAD * init_position(0),
                      iNav::DEGREE_2_RAD * init_position(1)));

  q_b_last_to_n_last_ = iNav::EulerAngle2Quat(iNav::DEGREE_2_RAD * init_theta);
}

/**
 * @brief destroy the INSMechanization::INS object
 */
INSMechanization::~INSMechanization() {}

/**
 * @brief sensor update
 *
 * @param time sensor update time
 * @param delta_theta gyro data
 * @param delta_v acc data
 */
void INSMechanization::SensorUpdate(iNav::IMUData data) {
  time_ = data.timestamp;
  delta_time_ = data.timestamp - last_time_;

  delta_theta_ = data.gyro;
  delta_v_ = data.acc;
}

/**
 * @brief INS update
 *
 * @return iNav::NavData: return INS update result
 */
iNav::NavData INSMechanization::MechanizationUpdate() {
  VelocityUpdate();
  PositionUpdate();
  AttitudeUpdate();

  iNav::NavData mechanization_output;
  mechanization_output.timestamp = time_;

  mechanization_output.att =
      iNav::RAD_2_DEGREE * iNav::DCM2EulerAngle(q_b_to_n_.toRotationMatrix());

  mechanization_output.vel = v_proj_n_;

  Eigen::Vector2d geodetic_vec =
      iNav::RAD_2_DEGREE * iNav::Quat2GeodeticVec(q_n_to_e_);
  mechanization_output.pos << geodetic_vec, h_;

  DataRecord();

  return mechanization_output;
}

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

  Eigen::Quaterniond q_n_midway_to_e_midway_ =
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

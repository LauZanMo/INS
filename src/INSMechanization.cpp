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
INSMechanization::INSMechanization(double init_time, Vector3d init_theta,
                                   Vector3d init_velocity, double init_phi,
                                   double init_lamda, double init_height,
                                   Vector3d init_delta_theta,
                                   Vector3d init_delta_velocity)
    : g_proj_n_(0, 0, 9.80665),
      omega_ie_proj_e_(0, 0, 7.2921151467e-5),
      last_time_(init_time),
      last_v_proj_n_(init_velocity),
      before_last_v_proj_n_(init_velocity),
      h_last_(init_height),
      last_delta_theta_(init_delta_theta),
      last_delta_v_(init_delta_velocity) {
  q_n_last_to_e_last_ =
      Quaterniond(cos(-pi_ / 4 - degree2rad_ * init_phi / 2) *
                      cos(degree2rad_ * init_lamda / 2),
                  -sin(-pi_ / 4 - degree2rad_ * init_phi / 2) *
                      sin(degree2rad_ * init_lamda / 2),
                  sin(-pi_ / 4 - degree2rad_ * init_phi / 2) *
                      cos(degree2rad_ * init_lamda / 2),
                  cos(-pi_ / 4 - degree2rad_ * init_phi / 2) *
                      sin(degree2rad_ * init_lamda / 2));

  q_b_last_to_n_last_ =
      Quaterniond(AngleAxisd(degree2rad_ * init_theta(2), Vector3d::UnitZ()) *
                  AngleAxisd(degree2rad_ * init_theta(1), Vector3d::UnitY()) *
                  AngleAxisd(degree2rad_ * init_theta(0), Vector3d::UnitX()));
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
void INSMechanization::SensorUpdate(double time, Vector3d delta_theta,
                                    Vector3d delta_v) {
  time_ = time;
  delta_time_ = time - last_time_;

  delta_theta_ = delta_theta;
  delta_v_ = delta_v;
}

/**
 * @brief INS update
 *
 * @return INSStorage: return INS update result
 */
INSStorage INSMechanization::MechanizationUpdate() {
  VelocityUpdate();
  PositionUpdate();
  AttitudeUpdate();

  INSStorage mechanization_output;
  mechanization_output.time_ = time_;

  Vector3d theta = rad2degree_ * ToEulerAngle(q_b_to_n_.matrix());
  mechanization_output.roll_ = theta(0);
  mechanization_output.pitch_ = theta(1);
  mechanization_output.yaw_ = theta(2);

  mechanization_output.v_n_ = v_proj_n_(0);
  mechanization_output.v_e_ = v_proj_n_(1);
  mechanization_output.v_d_ = v_proj_n_(2);

  Vector2d geodetic_vec =
      rad2degree_ * ToGeodeticVector(q_n_to_e_.toRotationMatrix());
  mechanization_output.phi_ = geodetic_vec(0);
  mechanization_output.lamda_ = geodetic_vec(1);
  mechanization_output.h_ = h_;

  DataRecord();

  return mechanization_output;
}

void INSMechanization::VelocityUpdate() {
  // update midway data
  v_midway_proj_n_ = 1.5 * last_v_proj_n_ - 0.5 * before_last_v_proj_n_;

  Vector3d zeta_midway = CalaulateZeta(last_v_proj_n_, q_n_last_to_e_last_,
                                       h_last_, 0.5 * delta_time_);
  Quaterniond q_n_midway_to_n_last(ToAngleAxis(zeta_midway));

  Vector3d xi_midway = omega_ie_proj_e_ * 0.5 * delta_time_;
  Quaterniond q_e_last_to_e_midway(ToAngleAxis(-xi_midway));

  Quaterniond q_n_midway_to_e_midway_ =
      q_e_last_to_e_midway * q_n_last_to_e_last_ * q_n_midway_to_n_last;

  h_midway_ = h_last_ - last_v_proj_n_(2) * 0.5 * delta_time_;

  // calculate delta_v_f_proj_n
  Vector3d zeta = CalaulateZeta(v_midway_proj_n_, q_n_midway_to_e_midway_,
                                h_midway_, delta_time_);

  Vector3d delta_v_f_proj_b_last = delta_v_ +
                                   0.5 * delta_theta_.cross(delta_v_) +
                                   1.0 / 12.0 *
                                       (last_delta_theta_.cross(delta_v_) +
                                        last_delta_v_.cross(delta_theta_));

  Vector3d delta_v_f_proj_n =
      (Matrix3d::Identity() - 0.5 * ToSkewSymmetricMat(zeta)) *
      q_b_last_to_n_last_.toRotationMatrix() * delta_v_f_proj_b_last;

  // calculate delta_v_g_and_coriolis_proj_n
  Vector3d delta_v_g_and_coriolis_proj_n =
      (g_proj_n_ - (2 * omega_ie_midway_proj_n_ + omega_en_midway_proj_n_)
                       .cross(v_midway_proj_n_)) *
      delta_time_;

  // calculate velocity
  v_proj_n_ = last_v_proj_n_ + delta_v_f_proj_n + delta_v_g_and_coriolis_proj_n;
}

void INSMechanization::PositionUpdate() {
  // update midway data
  v_midway_proj_n_ = 0.5 * (last_v_proj_n_ + v_proj_n_);

  // calculate height
  h_ = h_last_ - v_midway_proj_n_(2) * delta_time_;

  // calculate phi and lamda
  Vector3d zeta = CalaulateZeta(v_midway_proj_n_, q_n_midway_to_e_midway_,
                                h_midway_, delta_time_);
  Quaterniond q_n_to_n_last(ToAngleAxis(zeta));

  Vector3d xi = omega_ie_proj_e_ * delta_time_;
  Quaterniond q_e_last_to_e(ToAngleAxis(-xi));

  q_n_to_e_ = q_e_last_to_e * q_n_last_to_e_last_ * q_n_to_n_last;
}

void INSMechanization::AttitudeUpdate() {
  // update midway data
  h_midway_ = 0.5 * (h_last_ + h_);

  Quaterniond q_delta_theta = q_n_last_to_e_last_.inverse() * q_n_to_e_;
  AngleAxisd vec(q_delta_theta);
  vec.angle() = 0.5 * vec.angle();
  Quaterniond q_half_delta_theta(vec);
  q_n_midway_to_e_midway_ = q_n_last_to_e_last_ * q_half_delta_theta;

  // calculate q_b_to_b_last
  Vector3d phi =
      delta_theta_ + 1.0 / 12.0 * last_delta_theta_.cross(delta_theta_);

  Quaterniond q_b_to_b_last(ToAngleAxis(phi));

  // calculate q_n_last_to_n
  Vector3d zeta = CalaulateZeta(v_midway_proj_n_, q_n_midway_to_e_midway_,
                                h_midway_, delta_time_);
  Quaterniond q_n_last_to_n(ToAngleAxis(-zeta));

  // calculate theta
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

}  // namespace INS

/**
 * @file INS.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief INS class
 * @version 1.0
 * @date 2021-06-13
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include "INS.hpp"

namespace INS {

INS::INS(double init_time, Vector3d init_theta, Vector3d init_velocity,
         double init_phi, double init_lamda, double init_height,
         Vector3d init_delta_theta, Vector3d init_delta_velocity)
    : g_p_(0, 0, 9.80665),
      last_time_(init_time),
      last_theta_(AngleAxisd(degree2rad_ * init_theta[2], Vector3d::UnitZ()) *
                  AngleAxisd(degree2rad_ * init_theta[1], Vector3d::UnitY()) *
                  AngleAxisd(degree2rad_ * init_theta[0], Vector3d::UnitX())),
      last_velocity_(init_velocity),
      before_last_velocity_(init_velocity),
      last_position_(Vector3d(degree2rad_ * init_phi, degree2rad_ * init_lamda,
                              init_height)),
      last_delta_theta_(init_delta_theta),
      last_delta_velocity_(last_delta_velocity_) {
  double R_M = a_ * (1 - pow(e_, 2)) /
               sqrt(pow(1 - pow(e_ * sin(degree2rad_ * init_phi), 2), 3));
  double R_N = a_ / sqrt(1 - pow(e_ * sin(degree2rad_ * init_phi), 2));

  before_last_omega_ie_(0) = omega_e_ * cos(degree2rad_ * init_phi);
  before_last_omega_ie_(1) = 0;
  before_last_omega_ie_(2) = -omega_e_ * sin(degree2rad_ * init_phi);

  before_last_omega_en_(0) = init_velocity(1) / (R_N + init_height);
  before_last_omega_en_(1) = -init_velocity(0) / (R_M + init_height);
  before_last_omega_en_(2) =
      -init_velocity(1) * tan(degree2rad_ * init_phi) / (R_N + init_height);
}

INS::~INS() {}

void INS::SensorUpdate(double time, Vector3d delta_theta,
                       Vector3d delta_velocity) {
  time_ = time;
  delta_time_ = time - last_time_;

  delta_theta_ = delta_theta;
  delta_velocity_ = delta_velocity;
}

InsOutput INS::INSUpdate() {
  AttitudeUpdate();
  VelocityUpdate();
  PositionUpdate();

  InsOutput ins_output;
  ins_output.theta = rad2degree_ * theta_.matrix().eulerAngles(2, 1, 0);
  ins_output.velocity = velocity_;
  ins_output.position(0) = rad2degree_ * position_(0);
  ins_output.position(1) = rad2degree_ * position_(1);
  ins_output.position(2) = position_(2);

  DataRecord();

  return ins_output;
}

void INS::AttitudeUpdate() {
  // calculate q_b_to_b_last
  Vector3d phi =
      delta_theta_ +
      0.0833333333333333333 *
          last_theta_.matrix().eulerAngles(2, 1, 0).cross(delta_theta_);

  Quaterniond q_b_to_b_last(
      cos(0.5 * phi.norm()),
      sin(0.5 * phi.norm()) / (0.5 * phi.norm()) * (0.5 * phi(0)),
      sin(0.5 * phi.norm()) / (0.5 * phi.norm()) * (0.5 * phi(1)),
      sin(0.5 * phi.norm()) / (0.5 * phi.norm()) * (0.5 * phi(2)));

  // calculate q_n_last_to_n
  last_R_M_ = a_ * (1 - pow(e_, 2)) /
              sqrt(pow(1 - pow(e_ * sin(last_position_(0)), 2), 3));
  last_R_N_ = a_ / sqrt(1 - pow(e_ * sin(last_position_(0)), 2));

  last_omega_ie_(0) = omega_e_ * cos(last_position_(0));
  last_omega_ie_(1) = 0;
  last_omega_ie_(2) = -omega_e_ * sin(last_position_(0));

  last_omega_en_(0) = last_velocity_(1) / (last_R_N_ + last_position_(2));
  last_omega_en_(1) = -last_velocity_(0) / (last_R_M_ + last_position_(2));
  last_omega_en_(2) = -last_velocity_(1) * tan(last_position_(0)) /
                      (last_R_N_ + last_position_(2));

  Vector3d zeta = (last_omega_ie_ + last_omega_en_) * delta_time_;

  Quaterniond q_n_last_to_n(
      cos(0.5 * zeta.norm()),
      -sin(0.5 * zeta.norm()) / (0.5 * zeta.norm()) * (0.5 * zeta(0)),
      -sin(0.5 * zeta.norm()) / (0.5 * zeta.norm()) * (0.5 * zeta(1)),
      -sin(0.5 * zeta.norm()) / (0.5 * zeta.norm()) * (0.5 * zeta(2)));

  // calculate theta
  theta_ = q_n_last_to_n * last_theta_ * q_b_to_b_last;
}

void INS::VelocityUpdate() {
  // calculate delta_v_g_and_coriolis
  Vector3d middle_velocity = 1.5 * last_velocity_ - 0.5 * before_last_velocity_;
  Vector3d middle_omega_ie = 1.5 * last_omega_ie_ - 0.5 * before_last_omega_ie_;
  Vector3d middle_omega_en = 1.5 * last_omega_en_ - 0.5 * before_last_omega_en_;

  Vector3d delta_v_g_and_coriolis =
      (g_p_ - (2 * middle_omega_ie + middle_omega_en).cross(middle_velocity)) *
      delta_time_;

  // calculate delta_v_f
  Vector3d zeta_n_to_n_last = (middle_omega_ie + middle_omega_en) * delta_time_;

  Vector3d delta_v_f_from_b_last =
      delta_velocity_ + 0.5 * delta_theta_.cross(delta_velocity_) +
      0.0833333333333333333 * (last_delta_theta_.cross(delta_velocity_) +
                               last_delta_velocity_.cross(delta_theta_));

  Vector3d delta_v_f =
      (Matrix3d::Identity() - 0.5 * ToSkewSymmetricMat(zeta_n_to_n_last)) *
      last_theta_.toRotationMatrix() * delta_v_f_from_b_last;

  // calculate velocity
  velocity_ = last_velocity_ + delta_v_f + delta_v_g_and_coriolis;
}

void INS::PositionUpdate() {
  // calculate height
  position_(2) = last_position_(2) -
                 0.5 * (last_velocity_(2) + velocity_(2)) * delta_time_;

  // calculate phi
  double ave_height = 0.5 * (position_(2) + last_position_(2));

  position_(0) = last_position_(0) + 0.5 * (velocity_(0) + last_velocity_(0)) /
                                         (last_R_M_ + ave_height) * delta_time_;

  // calculate lamda
  double ave_phi = 0.5 * (position_(0) + last_position_(0));

  double R_N = a_ / sqrt(1 - pow(e_ * sin(position_(0)), 2));
  double middle_R_N = 0.5 * (last_R_N_ + R_N);

  position_(1) = last_position_(1) +
                 0.5 * (velocity_(1) + last_velocity_(1)) /
                     ((middle_R_N + ave_height) * cos(ave_phi)) * delta_time_;
}

void INS::DataRecord() {
  // time record
  last_time_ = time_;

  // IMU data record
  last_delta_theta_ = delta_theta_;
  last_delta_velocity_ = delta_velocity_;

  // INS data record
  last_theta_ = theta_;
  before_last_velocity_ = last_velocity_;
  last_velocity_ = velocity_;
  last_position_ = position_;
  before_last_omega_ie_ = last_omega_ie_;
  before_last_omega_en_ = last_omega_en_;
}

}  // namespace INS

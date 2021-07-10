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
#include "Utils.hpp"

// using namespace Eigen;

namespace INS {

class INSMechanization {
public:
  INSMechanization(const iNav::NavData& init_nav_data,
                   const iNav::IMUData& init_imu_data);

  ~INSMechanization();

  void SensorUpdate(iNav::IMUData data);
  iNav::NavData MechanizationUpdate();

protected:
  // IMU output
  Eigen::Vector3d delta_theta_, last_delta_theta_;
  Eigen::Vector3d delta_v_, last_delta_v_;

  // INS mechanization data
  double time_, last_time_, delta_time_;
  Eigen::Vector3d v_proj_n_, last_v_proj_n_, before_last_v_proj_n_;
  Eigen::Quaterniond q_n_to_e_, q_n_last_to_e_last_;
  double h_, h_last_;
  Eigen::Quaterniond q_b_to_n_, q_b_last_to_n_last_;

  Eigen::Vector3d omega_ie_midway_proj_n_;
  Eigen::Vector3d omega_en_midway_proj_n_;

  Eigen::Vector3d v_midway_proj_n_;
  Eigen::Quaterniond q_n_midway_to_e_midway_;
  double h_midway_;
  Eigen::Vector3d g_proj_n_;

private:
  // member function
  void VelocityUpdate();
  void PositionUpdate();
  void AttitudeUpdate();
  void DataRecord();

  Eigen::Vector3d ComputeZeta(const Eigen::Vector3d& v_proj_n,
                              const Eigen::Quaterniond& q_n_to_e,
                              const double& h, const double& delta_t);
  Eigen::Vector3d ComputeZeta(const Eigen::Vector3d& v_proj_n,
                              const Eigen::Quaterniond& q_n_to_e,
                              const double& h, const double& delta_t,
                              Eigen::Vector3d& omega_ie_proj_n,
                              Eigen::Vector3d& omega_en_proj_n);
};

}  // namespace INS

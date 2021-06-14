/**
 * @file main.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief main function
 * @version 1.0
 * @date 2021-06-13
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include <fstream>
#include <iostream>
#include <vector>

#include "DataTransform.hpp"
#include "INS.hpp"

using namespace std;

string read_path = "/home/ubuntu/Dataset/";
string imu_data_name = "IMU.bin";
string ref_data_name = "Reference.bin";

int main(int, char **) {
  // read files
  ifstream imu_data((read_path + imu_data_name).c_str(), ios::in | ios::binary);
  ifstream ref_data((read_path + ref_data_name).c_str(), ios::in | ios::binary);

  if (!imu_data || !ref_data) exit(-1);

  // parse data and save
  INS::IMUStorage imu_frame;
  INS::INSStorage ref_frame;
  vector<INS::IMUStorage> imu_vec;
  vector<INS::INSStorage> ref_vec;

  while (imu_data.read((char *)&imu_frame, sizeof(imu_frame)))
    imu_vec.push_back(imu_frame);
  while (ref_data.read((char *)&ref_frame, sizeof(ref_frame)))
    ref_vec.push_back(ref_frame);

  // initial state
  INS::INSStorage initial_state;
  initial_state.time_ = 91620.0;
  initial_state.phi_ = 23.1373950708;
  initial_state.lamda_ = 113.3713651222;
  initial_state.height_ = 2.175;
  initial_state.velocity_n_ = 0;
  initial_state.velocity_e_ = 0;
  initial_state.velocity_d_ = 0;
  initial_state.roll_ = 0.0107951084511778;
  initial_state.pitch_ = -2.14251290749072;
  initial_state.yaw_ = -75.7498049314083;

  // INS init and update
  INS::INS ins(
      initial_state.time_,
      Vector3d(initial_state.roll_, initial_state.pitch_, initial_state.yaw_),
      Vector3d(initial_state.velocity_n_, initial_state.velocity_e_,
               initial_state.velocity_d_),
      initial_state.phi_, initial_state.lamda_, initial_state.height_,
      Vector3d(imu_vec[0].gyro_x_, imu_vec[0].gyro_y_, imu_vec[0].gyro_z_),
      Vector3d(imu_vec[0].acc_x_, imu_vec[0].acc_y_, imu_vec[0].acc_z_));

  for (int i = 0; i < imu_vec.size(); i++) {
    ins.SensorUpdate(
        imu_vec[i].time_,
        Vector3d(imu_vec[i].gyro_x_, imu_vec[i].gyro_y_, imu_vec[i].gyro_z_),
        Vector3d(imu_vec[i].acc_x_, imu_vec[i].acc_y_, imu_vec[i].acc_z_));
    INS::InsOutput ins_output = ins.INSUpdate();
  }
}

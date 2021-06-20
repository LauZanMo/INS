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

#include "DataStorage.hpp"
#include "INSMechanization.hpp"

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
  initial_state.h_ = 2.175;
  initial_state.v_n_ = 0;
  initial_state.v_e_ = 0;
  initial_state.v_d_ = 0;
  initial_state.roll_ = 0.0107951084511778;
  initial_state.pitch_ = -2.14251290749072;
  initial_state.yaw_ = -75.7498049314083;

  INS::INSMechanization *ins_mechanization = nullptr;
  int begin;

  for (int i = 0; i < imu_vec.size(); i++) {
    if (imu_vec[i].time_ == initial_state.time_) {
      // INS init and update
      ins_mechanization = new INS::INSMechanization(
          initial_state.time_,
          Vector3d(initial_state.roll_, initial_state.pitch_,
                   initial_state.yaw_),
          Vector3d(initial_state.v_n_, initial_state.v_e_, initial_state.v_d_),
          initial_state.phi_, initial_state.lamda_, initial_state.h_,
          Vector3d(imu_vec[i].gyro_x_, imu_vec[i].gyro_y_, imu_vec[i].gyro_z_),
          Vector3d(imu_vec[i].acc_x_, imu_vec[i].acc_y_, imu_vec[i].acc_z_));
      begin = i;

    } else if (imu_vec[i].time_ > initial_state.time_ &&
               ins_mechanization != nullptr) {
      ins_mechanization->SensorUpdate(
          imu_vec[i].time_,
          Vector3d(imu_vec[i].gyro_x_, imu_vec[i].gyro_y_, imu_vec[i].gyro_z_),
          Vector3d(imu_vec[i].acc_x_, imu_vec[i].acc_y_, imu_vec[i].acc_z_));

      INS::INSStorage mechanization_output =
          ins_mechanization->MechanizationUpdate();

      cout.precision(12);
      cout << "my position: " << endl;
      cout << mechanization_output.phi_ << endl;
      cout << mechanization_output.lamda_ << endl;
      cout << mechanization_output.h_ << endl;

      cout << "ref position: " << endl;
      cout << ref_vec[i - begin].phi_ << endl;
      cout << ref_vec[i - begin].lamda_ << endl;
      cout << ref_vec[i - begin].h_ << endl;

      // cout << "my velocity: " << endl;
      // cout << mechanization_output.v_n_ << endl;
      // cout << mechanization_output.v_e_ << endl;
      // cout << mechanization_output.v_d_ << endl;

      // cout << "ref velocity: " << endl;
      // cout << ref_vec[i - begin].v_n_ << endl;
      // cout << ref_vec[i - begin].v_e_ << endl;
      // cout << ref_vec[i - begin].v_d_ << endl;

      // cout << "my attitude: " << endl;
      // cout << mechanization_output.roll_ << endl;
      // cout << mechanization_output.pitch_ << endl;
      // cout << mechanization_output.yaw_ << endl;

      // cout << "ref attitude: " << endl;
      // cout << ref_vec[i - begin].roll_ << endl;
      // cout << ref_vec[i - begin].pitch_ << endl;
      // cout << ref_vec[i - begin].yaw_ << endl;

      // cout << "v: " << ins_output.v << endl;
      // cout << "theta: " << ins_output.theta << endl;
    }
  }
}

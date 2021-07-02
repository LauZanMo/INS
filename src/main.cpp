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
  iNav::IMUData imu_frame;
  iNav::NavData ref_frame;
  vector<iNav::IMUData> imu_vec;
  vector<iNav::NavData> ref_vec;

  while (imu_data.read((char *)&imu_frame, sizeof(imu_frame)))
    imu_vec.push_back(imu_frame);
  while (ref_data.read((char *)&ref_frame, sizeof(ref_frame)))
    ref_vec.push_back(ref_frame);

  // initial state
  iNav::NavData initial_state;
  initial_state.timestamp = 91620.0;
  initial_state.pos = Eigen::Vector3d(23.1373950708, 113.3713651222, 2.175);
  initial_state.vel = Eigen::Vector3d(0, 0, 0);
  initial_state.att =
      Eigen::Vector3d(0.0107951084511778, -2.14251290749072, -75.7498049314083);

  INS::INSMechanization *ins_mechanization = nullptr;
  int begin;

  for (int i = 0; i < imu_vec.size(); i++) {
    if (imu_vec[i].timestamp == initial_state.timestamp) {
      // INS init and update
      ins_mechanization = new INS::INSMechanization(
          initial_state.timestamp, initial_state.pos, initial_state.vel,
          initial_state.att, imu_vec[i].gyro, imu_vec[i].acc);
      begin = i;

    } else if (imu_vec[i].timestamp > initial_state.timestamp &&
               ins_mechanization != nullptr) {
      ins_mechanization->SensorUpdate(imu_vec[i]);

      iNav::NavData mechanization_output =
          ins_mechanization->MechanizationUpdate();

      cout.precision(12);
      cout << "my position: " << endl;
      cout << mechanization_output.pos << endl;

      cout << "ref position: " << endl;
      cout << ref_vec[i - begin - 1].pos << endl;

      // cout << "my velocity: " << endl;
      // cout << mechanization_output.vel << endl;

      // cout << "ref velocity: " << endl;
      // cout << ref_vec[i - begin - 1].vel << endl;

      // cout << "my attitude: " << endl;
      // cout << mechanization_output.att << endl;

      // cout << "ref attitude: " << endl;
      // cout << ref_vec[i - begin-1].att << endl;
    }
  }
}

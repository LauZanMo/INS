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
string output_name = "mechanization_result.txt";

class RefData {
public:
  double timestamp;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d att;
};

int main(int, char **) {
  // read and write files
  ifstream imu_data((read_path + imu_data_name).c_str(), ios::in | ios::binary);
  ifstream ref_data((read_path + ref_data_name).c_str(), ios::in | ios::binary);
  ofstream result_data((read_path + output_name).c_str(), ios::trunc);

  if (!imu_data || !ref_data) exit(-1);

  // parse data and save
  iNav::IMUData imu_frame;
  RefData ref_frame;
  vector<iNav::IMUData> imu_vec;
  vector<RefData> ref_vec;

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
      ins_mechanization = new INS::INSMechanization(initial_state, imu_vec[i]);
      begin = i;

    } else if (imu_vec[i].timestamp > initial_state.timestamp &&
               ins_mechanization != nullptr) {
      ins_mechanization->SensorUpdate(imu_vec[i]);

      iNav::NavData mechanization_output =
          ins_mechanization->MechanizationUpdate();

      // cout.precision(12);
      // cout << "---" << endl;

      // cout << "my position: " << endl;
      // cout << mechanization_output.pos.transpose() << endl;

      // cout << "ref position: " << endl;
      // cout << ref_vec[i - begin - 1].pos.transpose() << endl;

      // cout << "my velocity: " << endl;
      // cout << mechanization_output.vel.transpose() << endl;

      // cout << "ref velocity: " << endl;
      // cout << ref_vec[i - begin - 1].vel.transpose() << endl;

      // cout << "my attitude: " << endl;
      // cout << mechanization_output.att.transpose() << endl;

      // cout << "ref attitude: " << endl;
      // cout << ref_vec[i - begin - 1].att.transpose() << endl;

      result_data.precision(12);
      result_data << mechanization_output.timestamp << " ";
      result_data << mechanization_output.pos[0] - ref_vec[i - begin - 1].pos[0]
                  << " "
                  << mechanization_output.pos[1] - ref_vec[i - begin - 1].pos[1]
                  << " "
                  << mechanization_output.pos[2] - ref_vec[i - begin - 1].pos[2]
                  << " ";
      result_data << mechanization_output.vel[0] - ref_vec[i - begin - 1].vel[0]
                  << " "
                  << mechanization_output.vel[1] - ref_vec[i - begin - 1].vel[1]
                  << " "
                  << mechanization_output.vel[2] - ref_vec[i - begin - 1].vel[2]
                  << " ";
      result_data << mechanization_output.att[0] - ref_vec[i - begin - 1].att[0]
                  << " "
                  << mechanization_output.att[1] - ref_vec[i - begin - 1].att[1]
                  << " "
                  << mechanization_output.att[2] - ref_vec[i - begin - 1].att[2]
                  << endl;
    }
  }
}

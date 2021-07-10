#include <fstream>
#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include "GINS.hpp"

using namespace std;

string read_path = "/home/ubuntu/Dataset/";
string imu_data_name = "A15_imu.bin";
string gnss_data_name = "GNSS_RTK.txt";
string truth_name = "truth.nav";

class Truth {
public:
  double week;
  double second;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d att;
};

int main(int argc, char const *argv[]) {
  // read files
  ifstream imu_data((read_path + imu_data_name).c_str(), ios::in | ios::binary);
  ifstream gnss_data((read_path + gnss_data_name).c_str());
  ifstream truth_data((read_path + truth_name).c_str());

  if (!imu_data || !gnss_data || !truth_data) exit(-1);

  // parse data and save
  iNav::IMUData imu_frame;
  iNav::GnssData gnss_frame;
  Truth truth_frame;
  vector<iNav::IMUData> imu_vec;
  vector<iNav::GnssData> gnss_vec;
  vector<Truth> truth_vec;

  iNav::NavData initial_state;
  initial_state.timestamp = 456300.0;
  initial_state.pos = Eigen::Vector3d(30.444787369, 114.471863247, 20.910);
  initial_state.pos_std = Eigen::Vector3d(0.005, 0.004, 0.008);
  initial_state.vel = Eigen::Vector3d(0, 0, 0);
  initial_state.vel_std = Eigen::Vector3d(0.003, 0.004, 0.004);
  initial_state.att = Eigen::Vector3d(0.854, -2.0345, 185.696);
  initial_state.att_std = Eigen::Vector3d(0.003, 0.003, 0.023);

  iNav::IMUParam imu_param;
  imu_param.ARW = 0.003;
  imu_param.VRW = 0.03;
  imu_param.gyro_bias_std = 0.027;
  imu_param.T_gyro_bias = 4;
  imu_param.acc_bias_std = 15;
  imu_param.T_acc_bias = 4;
  imu_param.gyro_scalar_std = 300;
  imu_param.T_gyro_scalar = 4;
  imu_param.acc_scalar_std = 300;
  imu_param.T_acc_scalar = 4;

  Eigen::Vector3d l_b(0.136, -0.301, -0.184);

  // save data in vector
  while (fabs(imu_frame.timestamp - initial_state.timestamp) > 0.001)
    imu_data.read((char *)&imu_frame, sizeof(imu_frame));
  imu_vec.push_back(imu_frame);

  while (truth_frame.second < initial_state.timestamp) {
    truth_data >> truth_frame.week >> truth_frame.second >>
        truth_frame.pos[0] >> truth_frame.pos[1] >> truth_frame.pos[2] >>
        truth_frame.vel[0] >> truth_frame.vel[1] >> truth_frame.vel[2] >>
        truth_frame.att[0] >> truth_frame.att[1] >> truth_frame.att[2];
  }
  truth_vec.push_back(truth_frame);

  while (gnss_frame.timestamp < initial_state.timestamp) {
    gnss_data >> gnss_frame.timestamp >> gnss_frame.pos[0] >>
        gnss_frame.pos[1] >> gnss_frame.pos[2] >> gnss_frame.pos_std[0] >>
        gnss_frame.pos_std[1] >> gnss_frame.pos_std[2];
  }
  gnss_vec.push_back(gnss_frame);

  while (imu_data.read((char *)&imu_frame, sizeof(imu_frame)))
    imu_vec.push_back(imu_frame);

  while (!gnss_data.eof()) {
    gnss_data >> gnss_frame.timestamp >> gnss_frame.pos[0] >>
        gnss_frame.pos[1] >> gnss_frame.pos[2] >> gnss_frame.pos_std[0] >>
        gnss_frame.pos_std[1] >> gnss_frame.pos_std[2];
    gnss_vec.push_back(gnss_frame);
  }

  while (!truth_data.eof()) {
    truth_data >> truth_frame.week >> truth_frame.second >>
        truth_frame.pos[0] >> truth_frame.pos[1] >> truth_frame.pos[2] >>
        truth_frame.vel[0] >> truth_frame.vel[1] >> truth_frame.vel[2] >>
        truth_frame.att[0] >> truth_frame.att[1] >> truth_frame.att[2];
    truth_vec.push_back(truth_frame);
  }

  iNav::GINS gnss_ins(initial_state, imu_param, l_b, imu_vec[0]);

  iNav::NavData nav_output;
  int i = 1;
  for (int j = 1; j < imu_vec.size(); j++) {
    if (imu_vec[j].timestamp < gnss_vec[i].timestamp) {
      nav_output = gnss_ins.Mechanization(imu_vec[j]);
      gnss_ins.Prediction();
    } else {
      nav_output = gnss_ins.GNSSUpdate(gnss_vec[i++], imu_vec[j]);
    }

    cout.precision(12);
    cout << "my position: " << endl;
    cout << nav_output.pos.transpose() << endl;
  }

  return 0;
}

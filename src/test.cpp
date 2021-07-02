#include "GINS.hpp"
#include "Utils.hpp"

#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;
using namespace std;

void Euler2Quat(const Eigen::Vector3d& euler, Eigen::Quaterniond& quat);
Vector2d ToGeodeticVector(const Matrix3d mat);
void Quat2DCM(const Eigen::Quaterniond& quat, Eigen::Matrix3d& dcm);

int main(int argc, char const* argv[]) {
  // test euler angle to quaternion
  Vector3d a(35, 56, 69);
  cout << (iNav::EulerAngle2Quat(a)).toRotationMatrix() << endl;
  Quaterniond b;
  Euler2Quat(a, b);
  cout << b.toRotationMatrix() << endl;

  cout << endl;
  cout << ToGeodeticVector(b.toRotationMatrix()) << endl;
  cout << iNav::Quat2GeodeticVec(b) << endl;

  cout << endl;
  cout << b.toRotationMatrix() << endl;
  Matrix3d c;
  Quat2DCM(b, c);
  cout << c << endl;

  cout << endl;
  Vector3d d(0, 3, 2);
  Quaterniond e(cos(0.5 * d.norm()),
                sin(0.5 * d.norm()) / (0.5 * d.norm()) * 0.5 * d(0),
                sin(0.5 * d.norm()) / (0.5 * d.norm()) * 0.5 * d(1),
                sin(0.5 * d.norm()) / (0.5 * d.norm()) * 0.5 * d(2));

  cout << e.toRotationMatrix() << endl;
  cout << Quaterniond(iNav::RotationVec2AngleAxis(d)).toRotationMatrix()
       << endl;

  cout << endl;
  Quaterniond f(cos(0.5 * d.norm()),
                -sin(0.5 * d.norm()) / (0.5 * d.norm()) * 0.5 * d(0),
                -sin(0.5 * d.norm()) / (0.5 * d.norm()) * 0.5 * d(1),
                -sin(0.5 * d.norm()) / (0.5 * d.norm()) * 0.5 * d(2));
  cout << f.toRotationMatrix() << endl;
  cout << Quaterniond(iNav::RotationVec2AngleAxis(-d)).toRotationMatrix()
       << endl;

  c = Vector3d(a.array().pow(2)).asDiagonal();
  cout << c << endl;

  cout.precision(32);

  cout << iNav::PI / 3600 / 180 << endl;
  cout << iNav::PI / 60 / 180 << endl;
  cout << 1 / (double)60.0 << endl;

  cout << a.array()/(1+a.array()) << endl;

  return 0;
}

void Euler2Quat(const Eigen::Vector3d& euler, Eigen::Quaterniond& quat) {
  double rol = euler[0];
  double pit = euler[1];
  double yaw = euler[2];
  quat.w() = cos(rol / 2) * cos(pit / 2) * cos(yaw / 2) +
             sin(rol / 2) * sin(pit / 2) * sin(yaw / 2);
  quat.x() = sin(rol / 2) * cos(pit / 2) * cos(yaw / 2) -
             cos(rol / 2) * sin(pit / 2) * sin(yaw / 2);
  quat.y() = cos(rol / 2) * sin(pit / 2) * cos(yaw / 2) +
             sin(rol / 2) * cos(pit / 2) * sin(yaw / 2);
  quat.z() = cos(rol / 2) * cos(pit / 2) * sin(yaw / 2) -
             sin(rol / 2) * sin(pit / 2) * cos(yaw / 2);
}

Vector2d ToGeodeticVector(const Matrix3d mat) {
  return Vector2d(atan2(-mat(2, 2), mat(2, 0)), atan2(-mat(0, 1), mat(1, 1)));
}

void Quat2DCM(const Eigen::Quaterniond& quat, Eigen::Matrix3d& dcm) {
  double q0 = quat.w();
  double q1 = quat.x();
  double q2 = quat.y();
  double q3 = quat.z();
  dcm << q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, 2 * (q1 * q2 - q0 * q3),
      2 * (q1 * q3 + q0 * q2), 2 * (q1 * q2 + q0 * q3),
      q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2 * (q2 * q3 - q0 * q2),
      2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1),
      q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}
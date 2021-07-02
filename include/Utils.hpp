#pragma once

#include <Eigen/Geometry>

namespace iNav {

/* Geometry */
const double PI = 3.141592653589793;
const double RAD_2_DEGREE = 57.29577951308232;
const double DEGREE_2_RAD = 0.017453292519943295;
const double DEGREE_PER_HOUR_2_RAD_PER_SECOND = 4.8481368110953e-06;
const double DEGREE_PER_SQRT_HOUR_2_RAD_PER_SQRT_SECOND = 0.0002908882086657;
const double METER_PER_SECOND_SQRT_HOUR_2_METER_PER_SECOND_SQRT_SECOND =
    0.0166666666666667;
const double MGAL_2_METER_PER_SECOND_SQUARE = 1e-5;
const double PPM_2_1 = 1e-6;
const double HOUR_2_SECOND = 3600;

const double OMEGA_E = 7.2921151467e-5;
const Eigen::Vector3d OMEGA_IE_PROJ_E(0, 0, OMEGA_E);
const double Ra = 6378137.0;
const double Rb = 6356752.3142;
const double E2 = 0.00669437999013;
const double GA = 9.7803267715;
const double GB = 9.8321863685;
const double F = 1.0 / 298.257223563;
const double GM = 3.986005e14;

inline Eigen::AngleAxisd RotationVec2AngleAxis(Eigen::Vector3d vec) {
  return Eigen::AngleAxisd(vec.norm(), vec.normalized());
}

inline Eigen::Quaterniond EulerAngle2Quat(const Eigen::Vector3d& angle) {
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitX()));
}

inline Eigen::Vector3d DCM2EulerAngle(const Eigen::Matrix3d& mat) {
  return Eigen::Vector3d(
      atan2(mat(2, 1), mat(2, 2)),
      atan2(-mat(2, 0), sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2))),
      atan2(mat(1, 0), mat(0, 0)));
}

inline Eigen::Vector2d Quat2GeodeticVec(const Eigen::Quaterniond& quat) {
  return Eigen::Vector2d(-2 * atan2(quat.y(), quat.w()) - PI / 2,
                         2 * atan2(quat.z(), quat.w()));
}

inline Eigen::Quaterniond GeodeticVec2Quat(const Eigen::Vector2d& vec) {
  return Eigen::Quaterniond(cos(-PI / 4 - vec[0] / 2) * cos(vec[1] / 2),
                            -sin(-PI / 4 - vec[0] / 2) * sin(vec[1] / 2),
                            sin(-PI / 4 - vec[0] / 2) * cos(vec[1] / 2),
                            cos(-PI / 4 - vec[0] / 2) * sin(vec[1] / 2));
}

inline Eigen::Vector2d ComputeRmRn(const double lat) {
  return Eigen::Vector2d(Ra * (1 - E2) / pow(1 - E2 * sin(lat) * sin(lat), 1.5),
                         Ra / sqrt(1 - E2 * sin(lat) * sin(lat)));
}

inline Eigen::Vector3d ComputeOmegaIEProjN(const double lat) {
  return Eigen::Vector3d(OMEGA_E * cos(lat), 0, -OMEGA_E * sin(lat));
}

inline Eigen::Vector3d ComputeOmegaENProjN(const Eigen::Vector3d v_proj_n,
                                           const double lat, const double h,
                                           const double R_M, const double R_N) {
  return Eigen::Vector3d(v_proj_n(1) / (R_N + h), -v_proj_n(0) / (R_M + h),
                         -v_proj_n(1) * tan(lat) / (R_N + h));
}

inline Eigen::Vector3d ComputeGn(const double lat, const double h) {
  double g_lat =
      (Ra * GA * pow(cos(lat), 2) + Rb * GB * pow(sin(lat), 2)) /
      sqrt(pow(Ra, 2) * pow(cos(lat), 2) + pow(Rb, 2) * pow(sin(lat), 2));
  double m = pow(OMEGA_E, 2) * pow(Ra, 2) * Rb / GM;

  return Eigen::Vector3d(
      0, 0,
      g_lat * (1 - 2.0 / Ra * (1 + F + m - 2 * F * pow(sin(lat), 2)) * h +
               3 * pow(h, 2) / pow(Ra, 2)));
}

/* Math */
inline Eigen::Matrix3d GetSkewSymmetricMat(const Eigen::Vector3d vec) {
  Eigen::Matrix3d vec_cross;
  vec_cross << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return vec_cross;
}

}  // namespace iNav

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <string>
#include <vector>

namespace rp {

using Vec3 = Eigen::Vector3d;
using VecX = Eigen::VectorXd;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Isometry = Eigen::Isometry3d;

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

inline double degToRad(double degrees) { return degrees * kPi / 180.0; }
inline double radToDeg(double radians) { return radians * 180.0 / kPi; }

inline Isometry makeIsometry(const Vec3& translation, const Mat3& rotation) {
  Isometry T = Isometry::Identity();
  T.linear() = rotation;
  T.translation() = translation;
  return T;
}

inline Mat3 rotationFromRpyDegrees(double roll_deg, double pitch_deg, double yaw_deg) {
  return (Eigen::AngleAxisd(degToRad(yaw_deg), Vec3::UnitZ()) *
          Eigen::AngleAxisd(degToRad(pitch_deg), Vec3::UnitY()) *
          Eigen::AngleAxisd(degToRad(roll_deg), Vec3::UnitX()))
      .toRotationMatrix();
}

inline Vec3 normalizedOr(const Vec3& value, const Vec3& fallback) {
  const double n = value.norm();
  if (n < kEpsilon) {
    return fallback;
  }
  return value / n;
}

inline bool isFinite(const VecX& q) {
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    if (!std::isfinite(q[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace rp

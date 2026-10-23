#pragma once

#include <cmath>

namespace gimbal_tracker {

constexpr double kPi = 3.14159265358979323846;

// Wrap an angle in radians to (-pi, pi].
inline double wrapPi(double angle) {
  constexpr double two_pi = 2.0 * kPi;
  angle = std::fmod(angle + kPi, two_pi);
  if (angle < 0.0) {
    angle += two_pi;
  }
  return angle - kPi;
}

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3 operator+(const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
  Vec3 operator-(const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
  Vec3 operator*(double scale) const { return {x * scale, y * scale, z * scale}; }
  Vec3 operator/(double scale) const { return {x / scale, y / scale, z / scale}; }

  double dot(const Vec3& other) const { return x * other.x + y * other.y + z * other.z; }
  double squaredNorm() const { return x * x + y * y + z * z; }
  double norm() const { return std::sqrt(squaredNorm()); }
};

inline Vec3 operator*(double scale, const Vec3& value) { return value * scale; }

// Unit boresight for pan (azimuth) and tilt (elevation, positive up).
inline Vec3 boresightDirection(double azimuth, double elevation) {
  const double ce = std::cos(elevation);
  return {ce * std::cos(azimuth), ce * std::sin(azimuth), std::sin(elevation)};
}

}  // namespace gimbal_tracker

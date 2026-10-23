#include "gimbal_tracker/lead.hpp"

#include "gimbal_tracker/angle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gimbal_tracker {
namespace {

struct BallisticAim {
  bool valid{false};
  double azimuth{0.0};
  double elevation{0.0};
  double time_of_flight{0.0};
};

Vec3 predictTarget(const LeadRequest& request, double time_s) {
  return request.target_position + request.target_velocity * time_s +
         request.target_acceleration * (0.5 * time_s * time_s);
}

BallisticAim solveBallistic(const Vec3& muzzle, const Vec3& point, double speed, double gravity) {
  BallisticAim aim;
  const Vec3 relative = point - muzzle;
  const double horizontal = std::hypot(relative.x, relative.y);
  const double z = relative.z;
  aim.azimuth = std::atan2(relative.y, relative.x);
  if (!(speed > 0.0)) {
    return aim;
  }

  if (gravity <= 1.0e-12) {
    const double distance = relative.norm();
    aim.valid = true;
    aim.time_of_flight = distance / speed;
    aim.elevation = std::atan2(z, horizontal);
    return aim;
  }

  if (horizontal < 1.0e-8) {
    // Shot is vertical. z = ±v t - 0.5 g t^2.
    const double discriminant = speed * speed - 2.0 * gravity * z;
    if (z >= 0.0) {
      if (discriminant < 0.0) {
        return aim;
      }
      const double time = (speed - std::sqrt(discriminant)) / gravity;
      if (time < 0.0) {
        return aim;
      }
      aim.valid = true;
      aim.elevation = 0.5 * kPi;
      aim.time_of_flight = time;
      return aim;
    }
    const double time = (-speed + std::sqrt(discriminant)) / gravity;
    if (time < 0.0) {
      return aim;
    }
    aim.valid = true;
    aim.elevation = -0.5 * kPi;
    aim.time_of_flight = time;
    return aim;
  }

  // z = R tan(el) - g R^2 / (2 v^2 cos^2(el))
  // with u = tan(el): c u^2 - R u + (z + c) = 0, c = g R^2 / (2 v^2).
  const double c = gravity * horizontal * horizontal / (2.0 * speed * speed);
  const double linear = -horizontal;
  const double constant = z + c;
  double discriminant = linear * linear - 4.0 * c * constant;
  const double disc_floor = -1.0e-9 * horizontal * horizontal;
  if (discriminant < disc_floor || !(c > 0.0)) {
    return aim;
  }
  discriminant = std::max(0.0, discriminant);
  const double root = std::sqrt(discriminant);
  const double u_hi = (-linear + root) / (2.0 * c);
  const double u_lo = (-linear - root) / (2.0 * c);

  auto consider = [&](double tangent) {
    const double elevation = std::atan(tangent);
    const double cosine = std::cos(elevation);
    if (cosine <= 1.0e-8) {
      return;
    }
    const double time = horizontal / (speed * cosine);
    if (time <= 0.0) {
      return;
    }
    if (!aim.valid || time < aim.time_of_flight) {
      aim.valid = true;
      aim.elevation = elevation;
      aim.time_of_flight = time;
    }
  };
  consider(u_hi);
  consider(u_lo);
  return aim;
}

LeadSolution solveKinematicClosedForm(const LeadRequest& request) {
  LeadSolution solution;
  const Vec3 relative = request.target_position - request.muzzle;
  const Vec3 velocity = request.target_velocity;
  const double speed = request.projectile_speed;
  const double a = velocity.squaredNorm() - speed * speed;
  const double b = 2.0 * relative.dot(velocity);
  const double c = relative.squaredNorm();

  if (c <= 1.0e-16) {
    solution.valid = true;
    solution.time_of_flight = 0.0;
    solution.intercept = request.target_position;
    solution.azimuth = 0.0;
    solution.elevation = 0.0;
    return solution;
  }

  double first = -1.0;
  double second = -1.0;
  if (std::abs(a) < 1.0e-12) {
    if (!(b < -1.0e-12)) {
      solution.reason = "target is not closing and matches projectile speed";
      return solution;
    }
    first = -c / b;
  } else {
    double discriminant = b * b - 4.0 * a * c;
    if (discriminant < -1.0e-9) {
      solution.reason = "target is unreachable at this projectile speed";
      return solution;
    }
    discriminant = std::max(0.0, discriminant);
    const double root = std::sqrt(discriminant);
    // Numerically stable roots of a t^2 + b t + c = 0.
    const double q = -0.5 * (b + std::copysign(root, b));
    if (std::abs(q) > 1.0e-15) {
      first = q / a;
      second = c / q;
    } else {
      first = (-b + root) / (2.0 * a);
      second = (-b - root) / (2.0 * a);
    }
  }

  double time = -1.0;
  for (double candidate : {first, second}) {
    if (candidate >= -1.0e-9 && (time < 0.0 || candidate < time)) {
      time = std::max(0.0, candidate);
    }
  }
  if (time < 0.0) {
    solution.reason = "intercept is in the past";
    return solution;
  }

  solution.valid = true;
  solution.time_of_flight = time;
  solution.intercept = predictTarget(request, time);
  const LookAngles look = lookAngles(request.muzzle, solution.intercept);
  solution.azimuth = look.azimuth;
  solution.elevation = look.elevation;
  return solution;
}

}  // namespace

LookAngles lookAngles(const Vec3& from, const Vec3& to) {
  const Vec3 relative = to - from;
  LookAngles angles;
  angles.range = relative.norm();
  angles.azimuth = std::atan2(relative.y, relative.x);
  angles.elevation = std::atan2(relative.z, std::hypot(relative.x, relative.y));
  return angles;
}

LeadSolution solveLead(const LeadRequest& request) {
  LeadSolution solution;
  if (!(request.projectile_speed > 0.0) || !std::isfinite(request.projectile_speed)) {
    solution.reason = "projectile speed must be positive";
    return solution;
  }
  if (request.gravity < 0.0) {
    solution.reason = "gravity must be non-negative";
    return solution;
  }

  const bool no_gravity = request.gravity <= 1.0e-12;
  const bool no_accel = request.target_acceleration.squaredNorm() <= 1.0e-18;
  if (no_gravity && no_accel) {
    return solveKinematicClosedForm(request);
  }

  LeadRequest kinematic = request;
  kinematic.gravity = 0.0;
  kinematic.target_acceleration = {0.0, 0.0, 0.0};
  const LeadSolution guess = solveKinematicClosedForm(kinematic);
  double time = guess.valid ? guess.time_of_flight
                            : (request.target_position - request.muzzle).norm() / request.projectile_speed;
  time = std::max(time, 1.0e-4);
  const int iterations = std::max(1, request.max_iterations);
  const double tolerance = std::max(request.tolerance, 1.0e-12);

  for (int i = 0; i < iterations; ++i) {
    const Vec3 intercept = predictTarget(request, time);
    if (no_gravity) {
      const double updated = (intercept - request.muzzle).norm() / request.projectile_speed;
      if (std::abs(updated - time) <= tolerance) {
        solution.valid = true;
        solution.time_of_flight = updated;
        solution.intercept = predictTarget(request, updated);
        const LookAngles look = lookAngles(request.muzzle, solution.intercept);
        solution.azimuth = look.azimuth;
        solution.elevation = look.elevation;
        return solution;
      }
      time = updated;
      continue;
    }

    const BallisticAim aim =
        solveBallistic(request.muzzle, intercept, request.projectile_speed, request.gravity);
    if (!aim.valid) {
      solution.reason = "predicted intercept is outside the ballistic envelope";
      return solution;
    }
    if (std::abs(aim.time_of_flight - time) <= tolerance) {
      solution.valid = true;
      solution.time_of_flight = aim.time_of_flight;
      solution.intercept = intercept;
      solution.azimuth = aim.azimuth;
      solution.elevation = aim.elevation;
      return solution;
    }
    time = aim.time_of_flight;
  }

  solution.reason = "lead iteration did not converge";
  return solution;
}

}  // namespace gimbal_tracker

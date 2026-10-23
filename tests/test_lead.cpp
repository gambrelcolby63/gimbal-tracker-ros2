#include "gimbal_tracker/gimbal_tracker.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using namespace gimbal_tracker;

Vec3 ballisticImpact(const LeadSolution& solution, double speed, double gravity) {
  const Vec3 direction = boresightDirection(solution.azimuth, solution.elevation);
  Vec3 impact = direction * (speed * solution.time_of_flight);
  impact.z -= 0.5 * gravity * solution.time_of_flight * solution.time_of_flight;
  return impact;
}

}  // namespace

TEST(Lead, StationaryTargetIsTimeOfFlightAlongTheLineOfSight) {
  LeadRequest request;
  request.target_position = {100.0, 0.0, 0.0};
  request.projectile_speed = 200.0;
  const LeadSolution solution = solveLead(request);
  ASSERT_TRUE(solution.valid);
  EXPECT_NEAR(solution.time_of_flight, 0.5, 1e-12);
  EXPECT_NEAR(solution.azimuth, 0.0, 1e-12);
  EXPECT_NEAR(solution.elevation, 0.0, 1e-12);
  EXPECT_NEAR((solution.intercept - request.target_position).norm(), 0.0, 1e-12);
}

TEST(Lead, CrossingTargetMeetsTheConstantSpeedConstraint) {
  LeadRequest request;
  request.target_position = {100.0, 0.0, 0.0};
  request.target_velocity = {0.0, 50.0, 0.0};
  request.projectile_speed = 200.0;
  const LeadSolution solution = solveLead(request);
  ASSERT_TRUE(solution.valid);
  const double expected_t = std::sqrt(10000.0 / 37500.0);
  EXPECT_NEAR(solution.time_of_flight, expected_t, 1e-9);
  EXPECT_NEAR(solution.intercept.y, 50.0 * expected_t, 1e-9);
  const double miss = (solution.intercept - request.muzzle).norm() -
                      request.projectile_speed * solution.time_of_flight;
  EXPECT_NEAR(miss, 0.0, 1e-8);
  EXPECT_NEAR(solution.azimuth, std::atan2(solution.intercept.y, solution.intercept.x), 1e-12);
}

TEST(Lead, HeadOnClosingTimeUsesTheSumOfSpeeds) {
  LeadRequest request;
  request.target_position = {100.0, 0.0, 0.0};
  request.target_velocity = {-10.0, 0.0, 0.0};
  request.projectile_speed = 200.0;
  const LeadSolution solution = solveLead(request);
  ASSERT_TRUE(solution.valid);
  EXPECT_NEAR(solution.time_of_flight, 100.0 / 210.0, 1e-9);
  EXPECT_NEAR(solution.azimuth, 0.0, 1e-12);
}

TEST(Lead, MuzzleOffsetChangesTheRelativeGeometry) {
  LeadRequest request;
  request.muzzle = {5.0, 0.0, 1.0};
  request.target_position = {5.0, 40.0, 1.0};
  request.projectile_speed = 80.0;
  const LeadSolution solution = solveLead(request);
  ASSERT_TRUE(solution.valid);
  EXPECT_NEAR(solution.time_of_flight, 0.5, 1e-12);
  EXPECT_NEAR(solution.azimuth, 0.5 * kPi, 1e-12);
  EXPECT_NEAR(solution.elevation, 0.0, 1e-12);
}

TEST(Lead, RejectsATargetThatOutrunsTheProjectile) {
  LeadRequest request;
  request.target_position = {10.0, 0.0, 0.0};
  request.target_velocity = {100.0, 0.0, 0.0};
  request.projectile_speed = 50.0;
  const LeadSolution solution = solveLead(request);
  EXPECT_FALSE(solution.valid);
  EXPECT_FALSE(solution.reason.empty());
}

TEST(Lead, RejectsANonPositiveProjectileSpeed) {
  LeadRequest request;
  request.target_position = {10.0, 0.0, 0.0};
  request.projectile_speed = 0.0;
  EXPECT_FALSE(solveLead(request).valid);
}

TEST(Lead, GravityLiftsTheElevationOfALevelShot) {
  LeadRequest request;
  request.target_position = {100.0, 0.0, 0.0};
  request.projectile_speed = 300.0;
  request.gravity = 9.81;
  request.tolerance = 1e-10;
  const LeadSolution solution = solveLead(request);
  ASSERT_TRUE(solution.valid);
  EXPECT_GT(solution.elevation, 0.0);
  EXPECT_LT(solution.elevation, 0.02);
  EXPECT_NEAR(solution.azimuth, 0.0, 1e-12);
  const Vec3 impact = ballisticImpact(solution, request.projectile_speed, request.gravity);
  EXPECT_NEAR(impact.x, 100.0, 1e-6);
  EXPECT_NEAR(impact.y, 0.0, 1e-6);
  EXPECT_NEAR(impact.z, 0.0, 1e-6);
}

TEST(Lead, GravityIterationHitsAMovingTarget) {
  LeadRequest request;
  request.target_position = {80.0, 20.0, 5.0};
  request.target_velocity = {10.0, -5.0, 1.0};
  request.projectile_speed = 250.0;
  request.gravity = 9.80665;
  request.tolerance = 1e-9;
  request.max_iterations = 16;
  const LeadSolution solution = solveLead(request);
  ASSERT_TRUE(solution.valid) << solution.reason;
  const Vec3 predicted = request.target_position + request.target_velocity * solution.time_of_flight;
  const Vec3 impact = ballisticImpact(solution, request.projectile_speed, request.gravity);
  EXPECT_NEAR((impact - solution.intercept).norm(), 0.0, 1e-5);
  EXPECT_NEAR((predicted - solution.intercept).norm(), 0.0, 1e-4);
}

TEST(Lead, BallisticEnvelopeRejectsAnImpossibleRange) {
  LeadRequest request;
  request.target_position = {5000.0, 0.0, 0.0};
  request.projectile_speed = 50.0;
  request.gravity = 9.81;
  EXPECT_FALSE(solveLead(request).valid);
}

TEST(Lead, AcceleratingTargetStillMeetsTheSpeedConstraint) {
  LeadRequest request;
  request.target_position = {40.0, -10.0, 2.0};
  request.target_velocity = {5.0, 8.0, 0.0};
  request.target_acceleration = {0.0, -1.5, 0.2};
  request.projectile_speed = 180.0;
  request.tolerance = 1e-9;
  const LeadSolution solution = solveLead(request);
  ASSERT_TRUE(solution.valid) << solution.reason;
  const Vec3 predicted = request.target_position + request.target_velocity * solution.time_of_flight +
                         request.target_acceleration * (0.5 * solution.time_of_flight * solution.time_of_flight);
  EXPECT_NEAR((predicted - solution.intercept).norm(), 0.0, 1e-6);
  EXPECT_NEAR((solution.intercept - request.muzzle).norm(),
              request.projectile_speed * solution.time_of_flight, 1e-6);
}

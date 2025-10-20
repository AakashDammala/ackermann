/**
 * @file src.cpp
 * @author Aakash Shetty Dammala (asd@umd.edu)
 * @brief Simple Ackermann steering controller.
 * @version 0.1
 * @date 2025-10-18
 * @copyright Copyright (c) 2025
 */

#include "libackermann.hpp"
#include <cmath>

// Clamp the value b/w low and high
template <typename T> static inline T clamp_val(T v, T lo, T hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static constexpr double RAD_TO_RPM = 60.0 / (2.0 * M_PI);

/**
 * @brief Construct the PIDController object (implementation).
 */
// Initialize PID controllers with configured gains and delta time
AckermannController::AckermannController(const AckermannConfig& config)
    : config_(config) {}

AckermannController::~AckermannController() {
  // no dynamic resources to free
}

/**
 * @brief Compute wheel RPMs and steering angles from commanded velocities.
 *
 * Kinematic model assumptions:
 * - Positive angular_vel implies turning left (counter-clockwise).
 * - Steering angles are limited by config_.steering_limits_.
 * - Linear wheel speeds are limited by config_.velocity_limits_.
 */
AckermannOutput AckermannController::compute(double linear_vel,
                                             double angular_vel,
                                             double current_time_sec) {
  AckermannOutput out{};

  // Stub implementation with fixed values
  out.steering_angle[0] = 1.0; // Left steering
  out.steering_angle[1] = 1.0; // Right steering

  // Fixed RPM values for all wheels
  out.wheel_rpm[0] = -10.0; // Front left
  out.wheel_rpm[1] = -20.0; // Front right
  out.wheel_rpm[2] = 30.0;  // Rear left
  out.wheel_rpm[3] = 40.0;  // Rear right

  out.clamped = false;
  return out;
}

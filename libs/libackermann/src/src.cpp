/**
 * @file src.cpp
 * @author Siddhant Deshmukh (iamsid@umd.edu)
 * @author Aakash Shetty Dammala (asd@umd.edu)
 * @brief Simple Ackermann steering controller.
 * @version 0.2
 * @date 2025-10-26
 * @copyright Copyright (c) 2025
 */

#include "libackermann.hpp"
#include <cmath>

// Clamp the value b/w low and high
template <typename T> static inline T clamp_val(T v, T lo, T hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static constexpr double RAD_TO_RPM = 60.0 / (2.0 * M_PI);
static constexpr double TWO_PI = 2.0 * M_PI;

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

AckermannModel::~AckermannModel() {
  // no dynamic resources to free
}

AckermannModel::AckermannModel(const AckermannConfig& config,
                                 AckermannState initial_state,const double delta_time)
    : ackermann_config_(config), state_(initial_state), delta_time_(delta_time) {}

AckermannState AckermannModel::update(double acceleration,
                                       double steering_angle) {
  state_.steering_angle_ = clamp_val(steering_angle, ackermann_config_.steering_limits_.first, ackermann_config_.steering_limits_.second);
  state_.heading_angle_ += state_.longitudnal_speed_ * delta_time_*std::tan(steering_angle)/ackermann_config_.drive_length_; // Update heading angle
  state_.heading_angle_ = fmod(state_.heading_angle_, TWO_PI);     // reduce angle into (-2π, 2π)
  if (state_.heading_angle_ < 0)
    state_.heading_angle_ += TWO_PI;            // wrap negative angles into [0, 2π)
  state_.longitudnal_speed_ += acceleration * delta_time_;
  state_.longitudnal_speed_ = clamp_val(state_.longitudnal_speed_, ackermann_config_.velocity_limits_.first, ackermann_config_.velocity_limits_.second);
  return state_;
}

// Convert linear speed (m/s) to wheel RPM
double AckermannModel::linearToRPM(double &linear_speed) const {
  return (linear_speed / ackermann_config_.wheel_radius_) * RAD_TO_RPM;
}

AckermannVehicleState AckermannModel::get_vehicle_state() const {
  const double eps = 1e-12;
  AckermannVehicleState vehicle_state{};
  double v_fl, v_fr, v_rl, v_rr; // Wheel linear speeds
  double linear_vel = state_.longitudnal_speed_;            // Linear speed
  double half_width = ackermann_config_.drive_width_ / 2.0;
  double L = ackermann_config_.drive_length_;
  double steering_left;
  double steering_right;

  if (std::abs(state_.steering_angle_) < eps ) {
    // Straight steering
    steering_left = 0.0;
    steering_right = 0.0;
    v_fl = v_fr = v_rl = v_rr = linear_vel;
  }
  else {
    double R = ackermann_config_.drive_length_ / std::tan(state_.steering_angle_ ); // Turning radius
    double angular_vel = linear_vel / R;                         // Angular velocity

    // Coordinates of front axle wheels relative to rear axle centerline
    // Using simple Ackermann steering: tan(theta) = L / (R +/- half_width)
    double R_left = R - half_width;
    double R_right = R + half_width;

    steering_left = std::atan2(L, R_left);
    steering_right = std::atan2(L, R_right);
 
  // Radii for each wheel (front left, front right, rear left, rear right)
    double R_fl = std::hypot(R_left, L);
    double R_fr = std::hypot(R_right, L);
    double R_rl = std::abs(R_left);
    double R_rr = std::abs(R_right);

    // Wheel linear speeds are proportional to radius around ICR
    v_fl = angular_vel * R_fl;
    v_fr = angular_vel * R_fr;
    v_rl = angular_vel * R_rl;
    v_rr = angular_vel * R_rr;
  }
  
  vehicle_state.wheel_steering_angle[0] = steering_left;
  vehicle_state.wheel_steering_angle[1] = steering_right;
  vehicle_state.wheel_rpm[0] = linearToRPM(v_fl); // Front left
  vehicle_state.wheel_rpm[1] = linearToRPM(v_fr); // Front
  vehicle_state.wheel_rpm[2] = linearToRPM(v_rl); // Rear left
  vehicle_state.wheel_rpm[3] = linearToRPM(v_rr); // Rear right
  
  return vehicle_state;

}
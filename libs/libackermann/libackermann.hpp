/**
 * @file libackermann.hpp
 * @author Aakash Shetty Dammala (asd@umd.edu)
 * @brief Simple Ackermann steering controller.
 * @version 0.1
 * @date 2025-10-18
 * @copyright Copyright (c) 2025
 */

#pragma once
#include <array>
#include <utility>
// #include "libpid.hpp"

/**
 * @brief Configuration for the Ackermann controller.
 */
struct AckermannConfig {
  double drive_width_;  /**< Distance between left and right wheels (m) */
  double drive_length_; /**< Distance between front and rear axles (m) */
  double wheel_radius_; /**< Wheel radius (m) */

  std::pair<double, double>
      velocity_limits_; /**< min and max linear velocity (m/s) */
  std::pair<double, double>
      steering_limits_; /**< min and max steering angle (rad) */
  // *** commented these variables for now, to clear cpp-check, will be uncommented later *** ///
  // PID parameters for wheel speed controllers (applied equally to all wheels)
  // double pid_Kp{0.0};
  // double pid_Ki{0.0};
  // double pid_Kd{0.0};
  // double pid_min_output{-1000.0};
  // double pid_max_output{1000.0};
  // double pid_delta_time{0.01};
};

/**
 * @brief Output of the Ackermann kinematics computation.
 *
 * Contains wheel speeds in RPM for four wheels (ordered: front_left,
 * front_right, rear_left, rear_right) and the steering angles (rad) for
 * the left and right front wheels respectively.
 */
struct AckermannOutput {
  double wheel_rpm[4];      /**< wheel speeds: FL, FR, RL, RR (RPM) */
  double steering_angle[2]; /**< steering angles: left, right (rad) */
  bool clamped{false};      /**< true if any output was clamped to limits */
};

class AckermannController {
public:
  /**
   * @brief Construct an AckermannController with the given configuration.
   * @param config Configuration (copied and stored as const member).
   */
  explicit AckermannController(const AckermannConfig& config);

  ~AckermannController();

  /**
   * @brief Compute wheel RPMs and steering angles from commanded velocities.
   *
   * @param linear_vel Commanded linear velocity (m/s).
   * @param angular_vel Commanded angular velocity (rad/s), positive = left
   * turn.
   * @return AckermannOutput containing 4 wheel RPMs and 2 steering angles.
   */
  static AckermannOutput compute(double linear_vel, double angular_vel,
                                 double current_time_sec);

private:
  const AckermannConfig config_;
};

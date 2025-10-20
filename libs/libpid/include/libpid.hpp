/**
 * @file libpid.hpp
 * @author Aakash Shetty Dammala (asd@umd.edu) - Part 1 (Driver)
 * @author Tirth Sadaria (tirths@umd.edu) - Part 1 (Navigator)
 * @author Siddhant Deshmukh (siddhantd@umd.edu) - Part 2 (Driver)
 * @author Grayson Gilbert (ggilbert@umd.edu) - Part 2 (Navigator)

 * @brief Simple PID controller interface.
 * @version 0.1
 * @date 2025-10-07
 * @version 0.2
 * @date 2025-10-14
 * @copyright Copyright (c) 2025
 */

#ifndef LIBPID_HPP_
#define LIBPID_HPP_

/**
 * @class PIDController
 * @brief Minimal PID controller for scalar inputs.
 */
class PIDController {
 public:
  /**
   * @brief Construct the PID controller.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param delta_time Control timestep in seconds.
   * @param min_output Minimum output (clamp lower bound).
   * @param max_output Maximum output (clamp upper bound).
   */
  PIDController(const double Kp, const double Ki, const double Kd,
                const double delta_time, const double min_output,
                const double max_output);

  /**
   * @brief Destroy the PIDController.
   */
  ~PIDController();

  /**
   * @brief Compute controller output for a single timestep.
   * @param target_setpoint Desired setpoint.
   * @param measured_value Current measurement.
   * @return double Controller output clamped to [min_output, max_output].
   */
  double compute(double target_setpoint, double measured_value);

 private:
  const double Kp_;          // proportional gain
  const double Ki_;          // integral gain
  const double Kd_;          // differential gain
  const double delta_time_;  // loop time of the controller
  const double min_output_;  // minimum output provided by the controller
  const double max_output_;  // maximum output provided by the controller
  double prev_error_;        // previous error value
  double integral_error_;    // cumulative integral error
  /**
   * @brief Compute the proportional term
   *
   * @param error Error value
   * @return double Controller proportional output
   */
  double compute_propotional(double error) const;  // Compute proportional term
  /**
   * @brief Compute the derivative term
   *
   * @param error Error value
   * @return double Controller derivative output
   */
  double compute_derivative(double error);  // Compute derivative term
  /**
   * @brief Compute the integral term
   *
   * @param error Error value
   * @return double Controller integral output
   */
  double compute_integral(double error);  // Compute integral term
};

#endif  // LIBPID_HPP_
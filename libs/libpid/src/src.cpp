/**
 * @file src.cpp
 * @author Aakash Shetty Dammala (asd@umd.edu)
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

#include "libpid.hpp"

PIDController::PIDController(const double Kp, const double Ki, const double Kd,
                             const double delta_time, const double min_output,
                             const double max_output)
    : Kp_(Kp),
      Ki_(Ki),
      Kd_(Kd),
      delta_time_(delta_time),
      min_output_(min_output),
      max_output_(max_output) {
  prev_error_ = 0.0;      // initialize previous error to zero
  integral_error_ = 0.0;  // initialize integral error to zero
  // initialize internal state if needed
}

PIDController::~PIDController() {
  // no dynamic resources to free
}

double PIDController::compute(double target_setpoint, double measured_value) {
  double error = target_setpoint - measured_value;

  double proportional_output = compute_propotional(error);  // P term
  double derivative_output = compute_derivative(error);     // D term
  double integral_output = compute_integral(error);         // I term

  double output = proportional_output + derivative_output +
                  integral_output;  // PID output before clamping

  // Clamp output to [min_output_, max_output_]
  if (output > max_output_) {
    output = max_output_;
  } else if (output < min_output_) {
    output = min_output_;
  }

  return output;
}

double PIDController::compute_propotional(double error) const {
  return Kp_ * error;  // P term;
}

double PIDController::compute_derivative(double error) {
  double derivative_output =
      Kd_ * (error - prev_error_) / delta_time_;  // D term
  prev_error_ = error;                            // Update previous error
  return derivative_output;
}

double PIDController::compute_integral(double error) {
  integral_error_ += error * delta_time_;  // Update integral error
  return Ki_ * integral_error_;            // I term
}

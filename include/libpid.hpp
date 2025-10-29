/**
 * @file libpid.hpp
 * @brief Scalar PID controller with anti-windup and derivative filtering
 * (CamelCase API).
 * @version 0.3
 * @date 2025-11-02
 *
 * @details
 * This header declares a minimal scalar PID controller suitable for embedded
 * and robotics control loops. Features:
 * - Proportional / Integral / Derivative terms
 * - First-order low-pass filter on the derivative term
 * - Integral clamping and back-calculation anti-windup
 * - Output clamping to a configured range
 *
 * Typical usage:
 * @code
 * PIDController pid(1.2, 0.6, 0.05, 0.02, -5.0, 5.0);
 * pid.SetAntiWindupGain(1.0);
 * pid.SetIntegralLimits(-2.0, 2.0);
 * pid.SetDerivativeFilterTau(0.03);
 * double u = pid.Compute(setpoint, measurement);
 * @endcode
 */

#ifndef LIBPID_HPP_
#define LIBPID_HPP_

/**
 * @class PIDController
 * @brief Scalar PID controller with optional anti-windup and derivative
 * filtering.
 *
 * @details
 * The controller computes:
 * \f[
 * u = \mathrm{sat}_{[u_{\min},u_{\max}]}\Big( K_p e
 * + I + K_d \,\hat{\dot e} \Big)
 * \f]
 * where \f$e = r - y\f$ is the error, \f$I\f$ is the clamped integral state,
 * and \f$\hat{\dot e}\f$ is a filtered derivative of the error.
 * Optionally, a back-calculation term feeds the saturation residual
 * \f$(u_{\text{sat}} - u)\f$ into the integrator with gain \f$K_{aw}\f$.
 */
class PIDController {
 public:
  /**
   * @brief Construct the PID controller.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param DeltaTime Fixed control step (seconds).
   * @param MinOutput Minimum output clamp.
   * @param MaxOutput Maximum output clamp.
   */
  PIDController(double Kp, double Ki, double Kd, double DeltaTime,
                double MinOutput, double MaxOutput);

  /**
   * @brief Configure anti-windup back-calculation gain.
   * @param Kaw Anti-windup gain (\f$\mathrm{s}^{-1}\f$-like scaling). Set 0 to
   * disable.
   *
   * @details
   * When enabled, the integrator receives \f$K_{aw}(u_{sat}-u)\f$ in addition
   * to the nominal integral update, reducing windup when outputs saturate.
   */
  void SetAntiWindupGain(double Kaw);

  /**
   * @brief Clamp the integral state to a fixed range.
   * @param IntegralMin Lower bound for the integral accumulator.
   * @param IntegralMax Upper bound for the integral accumulator.
   *
   * @details
   * Ensures integral state does not grow without bound.
   * If the current state is outside, it will be brought back into range.
   */
  void SetIntegralLimits(double IntegralMin, double IntegralMax);

  /**
   * @brief Set a first-order low-pass filter time constant for the derivative.
   * @param TauSeconds Time constant in seconds. Use 0 to disable filtering.
   *
   * @details
   * A larger \p TauSeconds yields more smoothing (and more lag) on the
   * derivative.
   */
  void SetDerivativeFilterTau(double TauSeconds);

  /**
   * @brief Gets the last computed proportional term.
   * @return double The last P term value.
   */
  double LastP() const noexcept { return LastPTerm; }

  /**
   * @brief Gets the last computed integral term.
   * @return double The last I term value.
   */
  double LastI() const noexcept { return LastITerm; }

  /**
   * @brief Gets the last computed derivative term.
   * @return double The last D term value.
   */
  double LastD() const noexcept { return LastDTerm; }

  /**
   * @brief Destroy the PID controller.
   */
  ~PIDController();

  /**
   * @brief Compute one PID step.
   * @param TargetSetpoint Desired setpoint \f$r\f$.
   * @param MeasuredValue  Current measurement \f$y\f$.
   * @return Output \f$u\f$ clamped to [MinOutput, MaxOutput].
   *
   * @details The method updates internal states (integral and derivative) and
   * applies output clamping and anti-windup if configured.
   */
  double Compute(double TargetSetpoint, double MeasuredValue);

 private:
  // --- Tunables and internal state (CamelCase; no underscores) ---

  /// Filtered derivative state \f$\hat{\dot e}\f$.
  double DerivativeState;

  /// Derivative low-pass filter time constant (seconds). 0 disables filtering.
  double TauDSeconds;

  /// Anti-windup back-calculation gain. 0 disables.
  double Kaw;

  /// Lower clamp for integral accumulator.
  double IntegralMin;

  /// Upper clamp for integral accumulator.
  double IntegralMax;

  /// Proportional gain.
  const double Kp;

  /// Integral gain.
  const double Ki;

  /// Derivative gain.
  const double Kd;

  /// Fixed control step (seconds).
  const double DeltaTime;

  /// Minimum output clamp.
  const double MinOutput;

  /// Maximum output clamp.
  const double MaxOutput;

  /// Previous error (for derivative).
  double PrevError;

  /// Integral accumulator (after clamping).
  double IntegralError;

  /// Last computed proportional term (for diagnostics).
  double LastPTerm{0.0};

  /// Last computed integral term (for diagnostics).
  double LastITerm{0.0};

  /// Last computed derivative term (for diagnostics).
  double LastDTerm{0.0};

  /**
   * @brief Compute proportional term.
   * @param Error Control error \f$e=r-y\f$.
   * @return \f$K_p e\f$.
   */
  double ComputeProportional(double Error) const;

  /**
   * @brief Compute derivative term (with optional filtering).
   * @param Error Control error \f$e=r-y\f$.
   * @return \f$K_d \hat{\dot e}\f$.
   */
  double ComputeDerivative(double Error);

  /**
   * @brief Compute integral term (with clamping).
   * @param Error Control error \f$e=r-y\f$.
   * @return Integral accumulator contribution.
   */
  double ComputeIntegral(double Error);
};

#endif  // LIBPID_HPP_

/**
 * @file libackermann.hpp
 * @author Siddhant Deshmukh (iamsid@umd.edu)
 * @author Aakash Shetty Dammala (asd@umd.edu)
 * @author Dayanidhi Kandade (dotsv@umd.edu)
 * @version 0.3
 * @date 2025-11-02
 * @brief PID-integrated Ackermann controller and kinematic model.
 *
 * @details
 * This module provides:
 * A lightweight kinematic model (@ref AckermannModel) for an Ackermann vehicle
 * that evolves the state and computes per-wheel outputs (RPM and steering
 * angles). A controller (@ref AckermannController) that converts vehicle-level
 * references — linear velocity *v* [m/s] and yaw rate *ω* [rad/s] — into
 * front-wheel steering angles and per-wheel RPM using two scalar PID
 * controllers (speed and steering).
 *
 * Conventions and assumptions:
 * - Positive angular velocity (ω > 0) implies a left/CCW turn.
 * - Front-axle bicycle model is used for steering geometry.
 * - Outputs are clamped to configured velocity and steering limits and may be
 * rate-limited and dead-banded.
 */

#pragma once
#include <array>
#include <cmath>
#include <utility>

#include "libpid.hpp"

/**
 * @namespace AckUtils
 * @brief Utility helpers for angle normalization (header-inline, no deps beyond
 * <cmath>).
 */
namespace AckUtils {

/**
 * @brief Wrap angle to the interval (-π, π].
 * @param a Angle in radians.
 * @return Angle wrapped to (-π, π].
 */
inline double WrapToPi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

/**
 * @brief Wrap angle to the interval [0, 2π).
 * @param a Angle in radians.
 * @return Angle wrapped to [0, 2π).
 */
inline double WrapTo2Pi(double a) {
  a = std::fmod(a, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a;
}

}  // namespace AckUtils

/**
 * @struct AckermannState
 * @brief Vehicle state for an Ackermann platform.
 *
 * @details
 * Holds the longitudinal body speed, the current heading (yaw) angle,
 * and the front-wheel steering angle that feeds the kinematic update.
 */
struct AckermannState {
  double LongitudinalSpeed;  ///< Body speed along x (m/s).
  double HeadingAngle;       ///< Heading / yaw (rad).
  double SteeringAngle;      ///< Front-wheel steering (rad).
};

/**
 * @struct AckermannVehicleState
 * @brief Per-wheel outputs for visualization/logging.
 *
 * @details
 * Contains individual wheel RPM (FL, FR, RL, RR) and the front left/right
 * steering angles (radians) computed from the current geometry and state.
 */
struct AckermannVehicleState {
  double WheelRpm[4];  ///< Wheel speeds: {FL, FR, RL, RR} in RPM.
  double
      WheelSteeringAngle[2];  ///< Front wheel steering: {left, right} in rad.
};

/**
 * @struct AckermannConfig
 * @brief Geometry, limits, and shaping parameters for controller/model.
 *
 * @details
 * - All lengths/distances are in meters.
 * - Limits are hard clamps applied to controller outputs.
 * - Rate limits are applied after clamping.
 * - Deadbands zero-out small commands around 0 to reduce chatter.
 */
struct AckermannConfig {
  double DriveWidth;   ///< Track width (m): lateral distance between left and
                       ///< right wheels.
  double DriveLength;  ///< Wheelbase (m): rear-to-front axle distance.
  double WheelRadius;  ///< Wheel radius (m).

  std::pair<double, double>
      VelocityLimits;  ///< Min/max longitudinal velocity (m/s).
  std::pair<double, double> SteeringLimits;  ///< Min/max steering angle (rad).

  double MaxVelocityRatePerSec{
      0.0};  ///< Max |Δv| per second (m/s²), 0 disables rate limiting.
  double MaxSteeringRatePerSec{
      0.0};  ///< Max |Δδ| per second (rad/s), 0 disables rate limiting.
  double VelocityDeadband{0.0};  ///< Deadband on commanded v (m/s).
  double SteeringDeadband{0.0};  ///< Deadband on commanded δ (rad).
};

/**
 * @class AckermannModel
 * @brief Simple bicycle-model kinematics with per-wheel mapping.
 *
 * @details
 * Responsibilities:
 * - Update the vehicle state with a fixed time step and steering input.
 * - Map the current motion/geometry to per-wheel linear speeds and steering
 * angles, then to RPM via wheel radius.
 *
 * The heading is typically kept wrapped using @ref AckUtils::WrapTo2Pi.
 */
class AckermannModel {
 public:
  /**
   * @brief Construct the model.
   * @param config       Geometry and limits (copied/stored).
   * @param initialState Initial vehicle state.
   * @param deltaTime    Fixed integration time step (seconds).
   */
  explicit AckermannModel(const AckermannConfig& config,
                          const AckermannState & initialState, double deltaTime);

  /**
   * @brief One integration step of the kinematic model.
   * @param acceleration Linear acceleration input (m/s²).
   * @param steeringAngle Front-wheel steering command (rad).
   * @return Updated @ref AckermannState after clamp and integration.
   *
   * @details
   * - Heading update (bicycle model): ψₖ₊₁ = wrap( ψₖ + vₖ Δt tan(δ) / L )
   * - Speed    update: vₖ₊₁ = clamp( vₖ + a Δt , [v_min, v_max] )
   * - Steering is clamped to [δ_min, δ_max].
   */
  AckermannState Update(double acceleration, double steeringAngle);

  /**
   * @brief Compute per-wheel RPM and front steering from current state.
   * @return @ref AckermannVehicleState with RPM (FL/FR/RL/RR) and front
   * left/right steering angles.
   */
  [[nodiscard]] AckermannVehicleState GetVehicleState() const noexcept;

  /// @brief Default destructor.
  ~AckermannModel() = default;

 private:
  const AckermannConfig Config;  ///< Immutable geometry/limits.
  AckermannState State;          ///< Current vehicle state.
  const double DeltaTime;        ///< Fixed time step (s).

  /**
   * @brief Convert linear wheel surface speed to RPM.
   * @param linearSpeed Linear speed along wheel perimeter (m/s).
   * @return Wheel revolutions per minute.
   */
  double LinearToRpm(double linearSpeed) const noexcept;
};

/**
 * @struct AckermannOutput
 * @brief Outputs of the controller compute stage.
 *
 * @details
 * Includes:
 * - Per-wheel RPM and front steering angles (left/right).
 * - Flags and echoes of the applied (post-limit/rate-limit) body commands.
 */
struct AckermannOutput {
  double WheelRpm[4];                 ///< Wheel RPM: {FL, FR, RL, RR}.
  double FrontWheelSteeringAngle[2];  ///< Front wheel steering: {left, right}
                                      ///< in rad.
  bool Clamped{false};  ///< True if any output hit a configured clamp.
  double VelocityCommandApplied{
      0.0};  ///< Applied longitudinal speed after shaping (m/s).
  double SteeringCommandApplied{
      0.0};  ///< Applied steering angle after shaping (rad).
};

/**
 * @class AckermannController
 * @brief Two-loop controller (speed + steering) that feeds Ackermann
 * kinematics.
 *
 * @details
 * Pipeline (per call to @ref Compute):
 * 1. Speed loop: PID on longitudinal speed using previously applied *v* as the
 * “measurement”; apply clamp → deadband → rate limit.
 * 2. Steering reference: δRef = atan2( L·ω , max(|v|, ε) ).
 * 3. Steering loop: PID on steering angle using previously applied δ as the
 * “measurement”; apply clamp → deadband → rate limit.
 * 4. Kinematics: Map (v, δ) to per-wheel RPM and front steering angles.
 *
 * PID implementations are injected (dependency injection) to keep the
 * controller testable and to decouple PID details from kinematic/geometry
 * logic.
 */
class AckermannController {
 public:
  /**
   * @brief Construct the controller with injected PIDs.
   * @param config   Geometry and limits (copied/stored).
   * @param velPid   PID for longitudinal speed (v).
   * @param steerPid PID for steering angle (δ).
   */
  AckermannController(const AckermannConfig& config, PIDController velPid,
                      PIDController steerPid);

  /// @brief Default destructor.
  ~AckermannController() = default;

  /**
   * @brief Compute per-wheel RPMs and front steering from vehicle-level
   * references.
   * @param linearVel      Commanded longitudinal speed *v* (m/s).
   * @param angularVel     Commanded yaw rate *ω* (rad/s), +CCW.
   * @param currentTimeSec Monotonic time (s) used to compute Δt for rate
   * limiting.
   * @return @ref AckermannOutput with per-wheel RPM, front steering, and
   * applied commands.
   *
   * @note If time is non-monotonic, a small default Δt is used internally.
   */
  [[nodiscard]] AckermannOutput Compute(double linearVel, double angularVel,
                                        double currentTimeSec) noexcept;

  /**
   * @brief Reset internal history (rate-limit state and last commands).
   *
   * @details
   * Clears cached last v/δ commands and stored time so the next call to
   * @ref Compute behaves like a fresh step.
   */
  void Reset() noexcept;

  /**
   * @brief (Optional) Provide external measurements to the controller.
   * @param measuredV      Measured longitudinal speed (m/s).
   * @param measuredDelta Measured steering angle (rad).
   *
   * @details
   * If you use external feedback instead of internal echoes, call this
   * to update the controller’s notion of “measurement”. Otherwise it’s a no-op.
   */
  void Observe(double measuredV, double measuredDelta) noexcept;

 private:
  /**
   * @brief Apply a symmetric deadband around zero.
   * @param x  Input value.
   * @param db Deadband half-width (same units as x).
   * @return 0 if |x| < db; otherwise returns x unchanged.
   */
  static inline double ApplyDeadband(double x, double db) {
    if (db <= 0.0) return x;
    return (std::abs(x) < db) ? 0.0 : x;
  }

  /**
   * @brief Dispatch to PID::Compute (shim kept for symmetry with earlier
   * variants).
   * @param pid      PID controller instance.
   * @param setpoint Desired setpoint value.
   * @param pv       Process value (measurement).
   * @return Controller output.
   */
  static double PidStep(PIDController& pid, double setpoint, double pv) {
    return pid.Compute(setpoint, pv);
  }

  const AckermannConfig Config;     ///< Immutable geometry/limits.
  PIDController VelocityPid;        ///< Speed loop PID (v).
  PIDController SteeringPid;        ///< Steering loop PID (δ).
  double LastVelocityCommand{0.0};  ///< Previously applied v (m/s).
  double LastSteeringCommand{0.0};  ///< Previously applied δ (rad).
  double LastTimestampSec{0.0};     ///< Previous time (s) for Δt.
};
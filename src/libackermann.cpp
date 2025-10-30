/**
 * @file libackermann.cpp
 * @author Siddhant Deshmukh (iamsid@umd.edu)
 * @author Aakash Shetty Dammala (asd@umd.edu)
 * @author Dayanidhi Kandade (dotsv@umd.edu)
 * @version 0.4
 * @date 2025-11-03
 * @copyright Copyright (c) 2025
 * @brief Runtime implementation for PID-integrated Ackermann controller and
 * model.
 *
 * @details
 * Implements:
 * @ref AckermannController: two-loop controller (speed + steering) with
 * clamping, deadband, optional rate limits, and kinematic mapping to per-wheel
 * outputs.
 * @ref AckermannModel: bicycle-model kinematics with per-wheel RPM/steering
 * angles.
 */

#include "libackermann.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

using AckUtils::WrapTo2Pi;

// Module constants
static constexpr double kRadToRpm = 60.0 / (2.0 * M_PI);
static constexpr double kTwoPi = 2.0 * M_PI;

namespace {
/**
 * @brief Internal struct to hold the results of kinematic calculations.
 */
struct WheelKinematics {
  double SteerLeft{0.0};   ///< Left front wheel steering angle (rad)
  double SteerRight{0.0};  ///< Right front wheel steering angle (rad)
  double V_FL{0.0};        ///< Front-left wheel linear velocity (m/s)
  double V_FR{0.0};        ///< Front-right wheel linear velocity (m/s)
  double V_RL{0.0};        ///< Rear-left wheel linear velocity (m/s)
  double V_RR{0.0};        ///< Rear-right wheel linear velocity (m/s)
};

/**
 * @brief Calculates per-wheel steering angles and linear velocities.
 * @details This is a core kinematic model, refactored from Compute() and
 * GetVehicleState() to avoid code duplication (a "bad smell" per Ch. 9/15
 * of the textbook). It implements the bicycle model equations for turning.
 *
 * @param L Wheelbase (DriveLength)
 * @param W Track width (DriveWidth)
 * @param deltaCmd The target steering angle (rad)
 * @param AccelCmd The target longitudinal velocity (m/s)
 * @return WheelKinematics struct with all calculated values.
 */
WheelKinematics CalculateWheelKinematics(const double L, const double W,
                                         const double deltaCmd,
                                         const double AccelCmd) {
  WheelKinematics kinematics;
  // Start with the default assumption: moving straight.
  kinematics.V_FL = AccelCmd;
  kinematics.V_FR = AccelCmd;
  kinematics.V_RL = AccelCmd;
  kinematics.V_RR = AccelCmd;

  // If not turning (delta ~ 0) or not moving (v ~ 0), all wheels are
  // straight and have the same speed.
  if (std::abs(deltaCmd) < 1e-9 || std::abs(AccelCmd) < 1e-12) {
    return kinematics;  // Return straight-motion values
  }

  // Calculate turning radius (R) from the bicycle model
  const double R = L / std::tan(deltaCmd);

  // Calculate angular velocity around the Instantaneous Center of Rotation
  // (ICR)
  const double wICR = AccelCmd / R;

  // Calculate turning radii for inner and outer wheels
  const double RLeft = R - 0.5 * W;
  const double RRight = R + 0.5 * W;

  // Calculate steering angles for inner and outer wheels (Ackermann steering)
  kinematics.SteerLeft = std::atan2(L, RLeft);
  kinematics.SteerRight = std::atan2(L, RRight);

  // Calculate linear velocities for all four wheels based on their
  // distance from the ICR.
  const double R_FL = std::hypot(RLeft, L);
  const double R_FR = std::hypot(RRight, L);
  const double R_RL = std::abs(RLeft);
  const double R_RR = std::abs(RRight);

  kinematics.V_FL = wICR * R_FL;
  kinematics.V_FR = wICR * R_FR;
  kinematics.V_RL = wICR * R_RL;
  kinematics.V_RR = wICR * R_RR;

  return kinematics;
}
}
// ============================== AckermannController ===============================

AckermannController::AckermannController(const AckermannConfig& config,
                                         PIDController velPid,
                                         PIDController steerPid)
    : Config(config),
      VelocityPid(std::move(velPid)),
      SteeringPid(std::move(steerPid)),
      LastVelocityCommand(0.0),
      LastSteeringCommand(0.0),
      LastTimestampSec(0.0) {}

void AckermannController::Reset() noexcept {
  LastVelocityCommand = 0.0;
  LastSteeringCommand = 0.0;
  LastTimestampSec = 0.0;
}

void AckermannController::Observe(double measuredV,
                                  double measuredDelta) noexcept {
  // Feeds the externally measured state back into the controller's
  // internal state. This is the "measurement" part of the feedback loop.
  LastVelocityCommand = measuredV;
  LastSteeringCommand = measuredDelta;
}

AckermannOutput AckermannController::Compute(double linearVel,
                                             double angularVel,
                                             double currentTimeSec) noexcept {
  AckermannOutput out{};

  // --- dt with monotonic guard ---
  // Ensure time delta is positive and reasonable, even if system clock jumps.
  const double dt = (currentTimeSec > LastTimestampSec)
                        ? (currentTimeSec - LastTimestampSec)
                        : 1e-3;  // Use a small default dt if time is invalid
  LastTimestampSec = currentTimeSec;

  // --- (1) Speed loop: PID on v (measurement = last applied) ---
  double vMeas = LastVelocityCommand;
  double AccelCmd = PidStep(VelocityPid, /*setpoint=*/linearVel, /*pv=*/vMeas);
  // Apply signal shaping: clamp, deadband, and rate limit
  AccelCmd = std::clamp(AccelCmd, Config.VelocityLimits.first,
                    Config.VelocityLimits.second);
  AccelCmd = ApplyDeadband(AccelCmd, Config.VelocityDeadband);
  if (Config.MaxVelocityRatePerSec > 0.0) {
    const double maxStep = Config.MaxVelocityRatePerSec * dt;
    const double step =
        std::clamp(AccelCmd - LastVelocityCommand, -maxStep, maxStep);
    AccelCmd = LastVelocityCommand + step;
  }
  LastVelocityCommand = AccelCmd;  // Cache the final applied command

  // --- (2) Steering reference from ω and v ---
  const double L = Config.DriveLength;
  // Use a minimum velocity when calculating steering angle to avoid instability
  // (atan2(0,0) is undefined, and small v causes large, fast angle changes)
  const double vForDelta =
      std::max(std::abs(linearVel), 0.2);  // robust at low speed
  double deltaRef = std::atan2(L * angularVel, vForDelta);

  // If nearly stopped, hold steering near last to avoid flopping
  if (std::abs(AccelCmd) < 0.1) {
    deltaRef = LastSteeringCommand;
  }

  // --- (3) Steering loop: PID on δ (measurement = last applied) ---
  double deltaMeas = LastSteeringCommand;
  double deltaCmd =
      PidStep(SteeringPid, /*setpoint=*/deltaRef, /*pv=*/deltaMeas);
  // Apply signal shaping: clamp, deadband, and rate limit
  deltaCmd = std::clamp(deltaCmd, Config.SteeringLimits.first,
                        Config.SteeringLimits.second);
  deltaCmd = ApplyDeadband(deltaCmd, Config.SteeringDeadband);
  if (Config.MaxSteeringRatePerSec > 0.0) {
    const double maxStep = Config.MaxSteeringRatePerSec * dt;
    const double step =
        std::clamp(deltaCmd - LastSteeringCommand, -maxStep, maxStep);
    deltaCmd = LastSteeringCommand + step;
  }
  LastSteeringCommand = deltaCmd;  // Cache the final applied command

  // --- (4) Kinematic mapping to per-wheel speeds/angles ---
  // This logic is shared with the AckermannModel and has been refactored
  // into a static helper to prevent code decay.
  const double W = Config.DriveWidth;
  const double r = Config.WheelRadius;

  WheelKinematics kinematics = CalculateWheelKinematics(L, W, deltaCmd, AccelCmd);

  out.FrontWheelSteeringAngle[0] = kinematics.SteerLeft;
  out.FrontWheelSteeringAngle[1] = kinematics.SteerRight;

  out.WheelRpm[0] = (kinematics.V_FL / r) * kRadToRpm;  // FL
  out.WheelRpm[1] = (kinematics.V_FR / r) * kRadToRpm;  // FR
  out.WheelRpm[2] = (kinematics.V_RL / r) * kRadToRpm;  // RL
  out.WheelRpm[3] = (kinematics.V_RR / r) * kRadToRpm;  // RR

  // Echo the final, shaped commands back to the caller
  out.VelocityCommandApplied = AccelCmd;
  out.SteeringCommandApplied = deltaCmd;
  out.Clamped = (AccelCmd == Config.VelocityLimits.first) ||
                (AccelCmd == Config.VelocityLimits.second) ||
                (deltaCmd == Config.SteeringLimits.first) ||
                (deltaCmd == Config.SteeringLimits.second);

  return out;
}

// ================================= AckermannModel ===============================

AckermannModel::AckermannModel(const AckermannConfig& config,
                               const AckermannState & initialState, double deltaTime)
    : Config(config), State(initialState), DeltaTime(deltaTime) {}

AckermannState AckermannModel::Update(double acceleration,
                                      double steeringAngle) {
  // Clamp steering to physical limits
  const double delta = std::clamp(steeringAngle, Config.SteeringLimits.first,
                                  Config.SteeringLimits.second);

  State.SteeringAngle = delta;

  // Heading integration with wrap to [0, 2π)
  // Only update heading if the vehicle is actually moving.
  if (std::abs(State.LongitudinalSpeed) > 1e-6) {
    State.HeadingAngle += State.LongitudinalSpeed * DeltaTime *
                          std::tan(delta) / Config.DriveLength;
    State.HeadingAngle = WrapTo2Pi(State.HeadingAngle);
  }

  // Speed integration with clamp to physical limits
  State.LongitudinalSpeed += acceleration * DeltaTime;
  State.LongitudinalSpeed =
      std::clamp(State.LongitudinalSpeed, Config.VelocityLimits.first,
                 Config.VelocityLimits.second);

  return State;
}

double AckermannModel::LinearToRpm(double linearSpeed) const noexcept {
  // Converts linear velocity (m/s) at the wheel's edge to RPM.
  return (linearSpeed / Config.WheelRadius) * kRadToRpm;
}

AckermannVehicleState AckermannModel::GetVehicleState() const noexcept {
  AckermannVehicleState vs{};

  const double v = State.LongitudinalSpeed;
  const double L = Config.DriveLength;
  const double W = Config.DriveWidth;

  // Call the refactored helper function to perform all kinematic calculations
  // based on the model's current state.
  WheelKinematics kinematics =
      CalculateWheelKinematics(L, W, State.SteeringAngle, v);

  vs.WheelSteeringAngle[0] = kinematics.SteerLeft;
  vs.WheelSteeringAngle[1] = kinematics.SteerRight;

  vs.WheelRpm[0] = LinearToRpm(kinematics.V_FL);
  vs.WheelRpm[1] = LinearToRpm(kinematics.V_FR);
  vs.WheelRpm[2] = LinearToRpm(kinematics.V_RL);
  vs.WheelRpm[3] = LinearToRpm(kinematics.V_RR);

  return vs;
}
/**
 * @file libpid.cpp
 * @author Aakash Shetty Dammala (asd@umd.edu)
 * @author Tirth Sadaria (tirths@umd.edu) - Part 1 (Navigator)
 * @author Siddhant Deshmukh (siddhantd@umd.edu) - Part 2 (Driver)
 * @author Grayson Gilbert (ggilbert@umd.edu) - Part 2 (Navigator)
 * @brief Simple PID controller interface.
 * @version 0.3
 * @date 2025-11-03
 * @copyright Copyright (c) 2025
 */

#include "libpid.hpp"

#include <algorithm>  // For std::clamp
#include <cmath>      // For std::abs

PIDController::PIDController(const double Kp, const double Ki, const double Kd, const double DeltaTime, const double MinOutput, const double MaxOutput)
    : DerivativeState(0.0),
      TauDSeconds(0.03),
      Kaw(0.0),
      IntegralMin(-1e6),
      IntegralMax(1e6),
      Kp(Kp),
      Ki(Ki),
      Kd(Kd),
      DeltaTime(DeltaTime),
      MinOutput(MinOutput),
      MaxOutput(MaxOutput),
      PrevError(0.0),
      IntegralError(0.0) {}

PIDController::~PIDController() {
  // no dynamic resources to free
}

void PIDController::SetAntiWindupGain(double Kaw) { Kaw = Kaw; }

void PIDController::SetIntegralLimits(double IntegralMin, double IntegralMax) {
  IntegralMin = IntegralMin;
  IntegralMax = IntegralMax;
}

void PIDController::SetDerivativeFilterTau(double TauSeconds) {
  TauDSeconds = (TauSeconds > 0.0) ? TauSeconds : 0.0;
}

double PIDController::Compute(double TargetSetpoint, double MeasuredValue) {
  const double error = TargetSetpoint - MeasuredValue;

  const double Up = ComputeProportional(error);
  const double Ui = ComputeIntegral(error);
  const double Ud = ComputeDerivative(error);

  // record for diagnostics
  LastPTerm = Up;
  LastITerm = Ui;  // Ui already includes Ki and clamping
  LastDTerm = Ud;

  const double u = Up + Ui + Ud;
  // Clamp the output to C++17 standard
  const double uSat = std::clamp(u, MinOutput, MaxOutput);

  // anti-windup back-calculation on the integral store
  if (Kaw != 0.0) {
    const double back = Kaw * (uSat - u);
    IntegralError += back;
    // enforce integral clamps again using C++17 std::clamp
    IntegralError = std::clamp(IntegralError, IntegralMin, IntegralMax);
  }

  PrevError = error;
  return uSat;
}

double PIDController::ComputeProportional(double Error) const {
  return Kp * Error;
}

double PIDController::ComputeDerivative(double Error) {
  const double de = (Error - PrevError) / DeltaTime;

  // Use a 1st-order low-pass filter on the derivative term
  // alpha = dt / (Tau + dt)
  const double alpha =
      (TauDSeconds > 1e-6)  // Avoid division by zero if Tau is tiny
          ? (DeltaTime / (TauDSeconds + DeltaTime))
          : 1.0;  // No filtering if Tau is zero

  // DerivativeState = (1-alpha)*DerivativeState + alpha*de
  DerivativeState += alpha * (de - DerivativeState);
  return Kd * DerivativeState;
}

double PIDController::ComputeIntegral(double Error) {
  // integrate with clamp on the integral store
  IntegralError += (Ki * Error * DeltaTime);
  IntegralError = std::clamp(IntegralError, IntegralMin, IntegralMax);
  return IntegralError;
}
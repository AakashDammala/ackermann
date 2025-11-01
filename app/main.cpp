/**
 * @file main.cpp
 * @brief CLI demo: PID-controlled Ackermann kinematics with CSV logging.
 * @author Dayanidhi Kandade (dotsv@umd.edu)
 * @version 0.3
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "libackermann.hpp"
#include "libpid.hpp"

// Simple PID factories (keep CamelCase and project defaults)
static PIDController MakeVelPid(double SampleTime = 0.02,
                                double MinOut = -10.0,
                                double MaxOut = 10.0) {
  PIDController pid(/*Kp=*/1.0, /*Ki=*/1.0, /*Kd=*/0.10,
                    /*SampleTime=*/SampleTime,
                    /*MinOutput=*/MinOut, /*MaxOutput=*/MaxOut);
  return pid;
}

static PIDController MakeSteerPid(double SampleTime = 0.02,
                                  double MinOut = -0.7,
                                  double MaxOut = 0.7) {
  PIDController pid(/*Kp=*/2.0, /*Ki=*/2.8, /*Kd=*/0.08,
                    /*SampleTime=*/SampleTime,
                    /*MinOutput=*/MinOut, /*MaxOutput=*/MaxOut);
  return pid;
}

int main(int argc, char** argv) {
  // Usage: <seconds> <dt> <v_ref> <w_ref> <csv_path> [--verbose]
  if (argc < 6) {
    std::cerr << "usage: " << argv[0]
              << " <seconds> <dt> <v_ref> <w_ref> <csv_path> [--verbose]\n";
    return 1;
  }

  const double SimulationSeconds = std::stod(argv[1]);
  const double SampleTime = std::stod(argv[2]);
  const double VRef       = std::stod(argv[3]);
  const double WRef       = std::stod(argv[4]);
  const std::string CsvPath = argv[5];
  const bool Verbose = (argc >= 7 && std::string(argv[6]) == "--verbose");

  // Vehicle configuration
  AckermannConfig cfg{};
  cfg.DriveWidth              = 0.6;
  cfg.DriveLength             = 1.0;
  cfg.WheelRadius             = 0.15;
  cfg.VelocityLimits          = {-2.0, 2.0};
  cfg.SteeringLimits          = {-0.7, 0.7};
  cfg.MaxVelocityRatePerSec   = 10.5;  // was MaxDvPerSec
  cfg.MaxSteeringRatePerSec   = 0.8;   // was MaxDdeltaPerSec
  cfg.VelocityDeadband        = 0.0;   // was VelDeadband
  cfg.SteeringDeadband        = 0.0;   // was SteerDeadband

  // Controllers
  PIDController velPid   = MakeVelPid(SampleTime, -5.0, 5.0);
  PIDController steerPid = MakeSteerPid(SampleTime, -0.7, 0.7);
  AckermannController ctrl(cfg, velPid, steerPid);

  // Plant state (for logging + feedback)
  AckermannState x{};
  x.LongitudinalSpeed = 0.0;
  x.HeadingAngle      = 0.0;
  x.SteeringAngle     = 0.0;
  AckermannModel plant(cfg, x, SampleTime);

  // Pose integrator for trajectory
  double X = 0.0, Y = 0.0;

  // CSV file
  std::ofstream csv(CsvPath);
  if (!csv) {
    std::cerr << "ERROR: cannot open CSV for write: " << CsvPath << "\n";
    return 2;
  }

  // Header: references, commands, measurements, errors, flags, wheels, pose
  csv << "t,"
      << "v_ref,w_ref,delta_ref,"
      << "v_cmd,delta_cmd,"
      << "v_meas,delta_meas,"
      << "v_err,delta_err,"
      << "clamped,vel_rate_limited,steer_rate_limited,"
      << "fl,fr,rl,rr,steer_l,steer_r,"
      << "heading,curvature,x,y\n";

  // Warm-up so first step uses positive dt
  (void)ctrl.Compute(VRef, WRef, 0.0);

  // Reference steering from references (constant target)
  const double VForDeltaRef = std::max(std::abs(VRef), 0.2);
  const double DeltaRef     = std::atan2(cfg.DriveLength * WRef, VForDeltaRef);

  // First-order speed plant constant
  const double SpeedPlantTau = 0.30;  // seconds

  // Track previous applied commands to infer rate limiting
  double PrevAccelCmd = 0.0, PrevDeltaCmd = 0.0;

  // Main loop
  int step = 0;
  for (double t = SampleTime; t <= SimulationSeconds + 1e-12; t += SampleTime, ++step) {
    const AckermannOutput out = ctrl.Compute(VRef, WRef, t);

    // Speed plant: vdot = (v_cmd - v)/tau
    const double Accel =
        (out.VelocityCommandApplied - x.LongitudinalSpeed) / SpeedPlantTau;
    x = plant.Update(Accel, out.SteeringCommandApplied);

    // Feedback actuals to the controller
    ctrl.Observe(x.LongitudinalSpeed, x.SteeringAngle);

    // External rate-limit flags from applied deltas
    const bool VelRateLimited =
        (cfg.MaxVelocityRatePerSec > 0.0) &&
        (std::abs(out.VelocityCommandApplied - PrevAccelCmd) >=
         cfg.MaxVelocityRatePerSec * SampleTime - 1e-9);

    const bool SteerRateLimited =
        (cfg.MaxSteeringRatePerSec > 0.0) &&
        (std::abs(out.SteeringCommandApplied - PrevDeltaCmd) >=
         cfg.MaxSteeringRatePerSec * SampleTime - 1e-9);

    // Curvature from measured steering
    const double Curvature = std::tan(x.SteeringAngle) / cfg.DriveLength;

    // Integrate pose
    X += x.LongitudinalSpeed * std::cos(x.HeadingAngle) * SampleTime;
    Y += x.LongitudinalSpeed * std::sin(x.HeadingAngle) * SampleTime;

    // Log one row
    csv << std::fixed << std::setprecision(8)
        << t << ','
        << VRef << ',' << WRef << ',' << DeltaRef << ','
        << out.VelocityCommandApplied << ',' << out.SteeringCommandApplied << ','
        << x.LongitudinalSpeed << ',' << x.SteeringAngle << ','
        << (VRef - x.LongitudinalSpeed) << ',' << (DeltaRef - x.SteeringAngle) << ','
        << (out.Clamped ? 1 : 0) << ','
        << (VelRateLimited ? 1 : 0) << ','
        << (SteerRateLimited ? 1 : 0) << ','
        << out.WheelRpm[0] << ',' << out.WheelRpm[1] << ','
        << out.WheelRpm[2] << ',' << out.WheelRpm[3] << ','
        << out.FrontWheelSteeringAngle[0] << ',' << out.FrontWheelSteeringAngle[1] << ','
        << x.HeadingAngle << ',' << Curvature << ','
        << X << ',' << Y << '\n';

    if (Verbose && (step % static_cast<int>(0.5 / SampleTime) == 0)) {
      std::cout << std::fixed << std::setprecision(3)
                << "t=" << t
                << " v_ref=" << VRef
                << " v_cmd=" << out.VelocityCommandApplied
                << " v_meas=" << x.LongitudinalSpeed
                << " w_ref=" << WRef
                << " delta_ref=" << DeltaRef
                << " delta_cmd=" << out.SteeringCommandApplied
                << " delta_meas=" << x.SteeringAngle
                << " clamp=" << out.Clamped
                << " rate[v,δ]=(" << VelRateLimited << "," << SteerRateLimited << ")\n";
    }

    PrevAccelCmd     = out.VelocityCommandApplied;
    PrevDeltaCmd = out.SteeringCommandApplied;
  }

  csv.close();
  std::cout << "Wrote CSV: " << CsvPath << "\n";
  return 0;
}


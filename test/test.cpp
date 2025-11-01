/**
 * @file test.cpp
 * @brief Unit tests for PID-integrated Ackermann controller and kinematic
 * model.
 *
 * @details
 * This suite validates:
 * - Controller: straight motion, turning geometry, clamping
 * (velocity/steering), tiny-omega straight handling, reverse motion, and
 * returned telemetry fields.
 * - Model: state updates (heading wrap, speed integration), wheel RPMs and
 *   steering angles for both straight and turning cases, and limit clamping.
 *
 * Each test includes a short “Manual calculation / expectation” note where
 * useful.
 *
 * @copyright
 * (c) 2025. Licensed under the project’s license.
 */

#include <gtest/gtest.h>

#include <cmath>

#include "libackermann.hpp"
#include "libpid.hpp"

using ::testing::Test;

/**
 * @brief Factory for a velocity-loop PID used in tests.
 *
 * @param dt    Controller time step (s).
 * @param minv  Minimum controller output (m/s).
 * @param maxv  Maximum controller output (m/s).
 * @return PIDController configured as P-only by default; output clamped to
 * [minv, maxv].
 */
static PIDController MakeVelPid(double dt = 0.02, double minv = -5.0,
                                double maxv = 5.0) {
  return PIDController(/*Kp=*/3.0, /*Ki=*/0.0, /*Kd=*/0.0,
                       /*delta_time=*/dt, /*min_output=*/minv,
                       /*max_output=*/maxv);
}

/**
 * @brief Factory for a steering-loop PID used in tests.
 *
 * @param dt    Controller time step (s).
 * @param mind  Minimum steering command (rad).
 * @param maxd  Maximum steering command (rad).
 * @return PIDController configured as P-only by default; output clamped to
 * [mind, maxd].
 */
static PIDController MakeSteerPid(double dt = 0.02, double mind = -0.7,
                                  double maxd = 0.7) {
  return PIDController(/*Kp=*/3.0, /*Ki=*/0.0, /*Kd=*/0.0,
                       /*delta_time=*/dt, /*min_output=*/mind,
                       /*max_output=*/maxd);
}

#define make_vel_pid MakeVelPid
#define make_steer_pid MakeSteerPid

/**
 * @test StraightLineAllWheelsSameRPM
 * @brief Straight-line command yields equal RPMs and ~zero steering.
 *
 * @details
 * Manual check (magnitude): \f$\text{RPM} = \frac{v}{r}\cdot\frac{60}{2\pi}\f$.
 * With returned @c VelocityCommandApplied and @c WheelRadius this is asserted
 * approximately.
 */
TEST(AckermannTest, StraightLineAllWheelsSameRPM) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.6;
  cfg.DriveLength = 1.0;
  cfg.WheelRadius = 0.15;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-1.57, 1.57};

  AckermannController ctrl(cfg, make_vel_pid(), make_steer_pid());
  // Bootstrap time so dt > 0:
  (void)ctrl.Compute(2.0, 0.0, 0.00);
  auto out = ctrl.Compute(2.0, 0.0, 0.01);  // 2 m/s straight at t=0.01 s

  // All wheel RPMs equal for straight motion
  EXPECT_NEAR(out.WheelRpm[0], out.WheelRpm[1], 1e-6);
  EXPECT_NEAR(out.WheelRpm[1], out.WheelRpm[2], 1e-6);
  EXPECT_NEAR(out.WheelRpm[2], out.WheelRpm[3], 1e-6);

  // Steering angles near zero
  EXPECT_NEAR(out.FrontWheelSteeringAngle[0], 0.0, 1e-3);
  EXPECT_NEAR(out.FrontWheelSteeringAngle[1], 0.0, 1e-3);

  // Magnitude check ≈ (v/r)*60/(2π)
  const double RadToRpm = 60.0 / (2.0 * M_PI);
  const double ExpectedRpm =
      (out.VelocityCommandApplied / cfg.WheelRadius) * RadToRpm;
  EXPECT_NEAR(out.WheelRpm[0], ExpectedRpm, 1e-3);
}

/**
 * @test TurningProducesDifferentWheelSpeedsAndSteering
 * @brief Left turn yields different steering angles and different inner/outer
 * RPMs.
 *
 * @details
 * Manual sketch: \f$R = v/\omega\f$; with axle half-width \f$W/2\f$,
 * \f$R_{L/R} = R \mp W/2\f$; steering \f$\delta_{L/R} = \arctan(L/R_{L/R})\f$;
 * wheel speeds scale with wheel radius to ICR.
 */
TEST(AckermannTest, TurningProducesDifferentWheelSpeedsAndSteering) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.6;
  cfg.DriveLength = 1.0;
  cfg.WheelRadius = 0.15;
  cfg.VelocityLimits = {-10.0, 10.0};
  cfg.SteeringLimits = {-1.57, 1.57};

  AckermannController ctrl(cfg, make_vel_pid(/*dt=*/0.02, -10, 10),
                           make_steer_pid(/*dt=*/0.02, -1.57, 1.57));
  // Bootstrap:
  (void)ctrl.Compute(1.0, 0.0, 0.00);

  const double v = 1.0;  // m/s
  const double w = 0.5;  // rad/s (positive => left turn)
  auto out = ctrl.Compute(v, w, 0.02);

  EXPECT_GT(std::abs(out.FrontWheelSteeringAngle[0]), 0.0);
  EXPECT_GT(std::abs(out.FrontWheelSteeringAngle[1]), 0.0);

  // Not all equal
  bool all_equal = (std::abs(out.WheelRpm[0] - out.WheelRpm[1]) < 1e-6) &&
                   (std::abs(out.WheelRpm[1] - out.WheelRpm[2]) < 1e-6);
  EXPECT_FALSE(all_equal);

  // Left turn: inner (left) wheels slower than right
  EXPECT_LT(std::abs(out.WheelRpm[0]), std::abs(out.WheelRpm[1]));  // FL < FR
  EXPECT_LT(std::abs(out.WheelRpm[2]), std::abs(out.WheelRpm[3]));  // RL < RR
}

/**
 * @test ZeroVelocitiesProduceZeroOutput
 * @brief Zero linear and angular velocity commands produce zero RPMs and zero
 * steering.
 */
TEST(AckermannTest, ZeroVelocitiesProduceZeroOutput) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.6;
  cfg.DriveLength = 1.0;
  cfg.WheelRadius = 0.15;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-1.57, 1.57};

  AckermannController ctrl(cfg, make_vel_pid(), make_steer_pid());
  auto out = ctrl.Compute(0.0, 0.0, 0.0);

  for (int i = 0; i < 4; ++i) EXPECT_NEAR(out.WheelRpm[i], 0.0, 1e-9);
  EXPECT_NEAR(out.FrontWheelSteeringAngle[0], 0.0, 1e-9);
  EXPECT_NEAR(out.FrontWheelSteeringAngle[1], 0.0, 1e-9);
}

/**
 * @test ReverseStraightAllWheelsSameRPM
 * @brief Negative linear command (reverse) preserves equality of RPM magnitudes
 * with negative sign.
 *
 * @details
 * Manual: RPM magnitude follows \f$|v|/r\cdot 60/(2\pi)\f$, sign tracks the
 * sign of \f$v\f$.
 */
TEST(AckermannTest, ReverseStraightAllWheelsSameRPM) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.6;
  cfg.DriveLength = 1.0;
  cfg.WheelRadius = 0.15;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-1.57, 1.57};

  AckermannController ctrl(cfg, make_vel_pid(), make_steer_pid());
  // Bootstrap:
  [[maybe_unused]] AckermannOutput warmup1 = ctrl.Compute(-3.0, 0.0, 0.00);
  auto out = ctrl.Compute(-3.0, 0.0, 0.01);

  EXPECT_NEAR(out.WheelRpm[0], out.WheelRpm[1], 1e-6);
  EXPECT_NEAR(out.WheelRpm[1], out.WheelRpm[2], 1e-6);
  EXPECT_NEAR(out.WheelRpm[2], out.WheelRpm[3], 1e-6);

  const double RAD_TO_RPM = 60.0 / (2.0 * M_PI);
  const double expected =
      (out.VelocityCommandApplied / cfg.WheelRadius) * RAD_TO_RPM;
  EXPECT_NEAR(out.WheelRpm[0], expected, 1e-3);
}

/**
 * @test VelocityLimitsCauseClamping
 * @brief Linear command above configured limit clamps to vmax.
 *
 * @details
 * Manual: with @c limits={-0.5,0.5}, any larger request clamps to 0.5 m/s and
 * RPM magnitude follows \f$0.5/r\cdot 60/(2\pi)\f$.
 */
TEST(AckermannTest, VelocityLimitsCauseClamping) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.6;
  cfg.DriveLength = 1.0;
  cfg.WheelRadius = 0.15;
  cfg.VelocityLimits = {-0.5, 0.5};  // tight
  cfg.SteeringLimits = {-1.57, 1.57};

  AckermannController ctrl(cfg, make_vel_pid(/*dt=*/0.02, -0.5, 0.5),
                           make_steer_pid());
  // Bootstrap:
  [[maybe_unused]] AckermannOutput warmup2 = ctrl.Compute(2.0, 0.0, 0.00);
  auto out = ctrl.Compute(2.0, 0.0, 0.01);

  const double RAD_TO_RPM = 60.0 / (2.0 * M_PI);
  const double expected =
      (out.VelocityCommandApplied / cfg.WheelRadius) * RAD_TO_RPM;
  EXPECT_NEAR(out.VelocityCommandApplied, 0.5, 1e-12);
  EXPECT_NEAR(out.WheelRpm[0], expected, 1e-3);
  EXPECT_TRUE(out.Clamped);
}

/**
 * @test SteeringLimitsCauseClamping
 * @brief Steering command that would exceed bounds is clamped to configured
 * limits.
 */
TEST(AckermannTest, SteeringLimitsCauseClamping) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.6;
  cfg.DriveLength = 1.0;
  cfg.WheelRadius = 0.15;
  cfg.VelocityLimits = {-10.0, 10.0};
  cfg.SteeringLimits = {-0.2, 0.2};  // tight

  AckermannController ctrl(cfg, make_vel_pid(/*dt=*/0.02, -10, 10),
                           make_steer_pid(/*dt=*/0.02, -0.2, 0.2));

  // Bootstrap:
  (void)ctrl.Compute(1.0, 0.0, 0.00);
  // Large ω to force clamp:
  auto out = ctrl.Compute(1.0, 5.0, 0.01);

  EXPECT_LE(out.SteeringCommandApplied, cfg.SteeringLimits.second + 1e-9);
  EXPECT_GE(out.SteeringCommandApplied, cfg.SteeringLimits.first - 1e-9);
  EXPECT_TRUE(out.Clamped);
}

/**
 * @test TinyAngularVelocityTreatedAsStraight
 * @brief Very small angular velocity is treated as straight (steering ≈ 0).
 */
TEST(AckermannTest, TinyAngularVelocityTreatedAsStraight) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.6;
  cfg.DriveLength = 1.0;
  cfg.WheelRadius = 0.15;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-1.57, 1.57};

  AckermannController ctrl(cfg, make_vel_pid(), make_steer_pid());
  // Bootstrap:
  [[maybe_unused]] AckermannOutput warmup3 = ctrl.Compute(1.0, 0.0, 0.00);
  auto out = ctrl.Compute(1.0, 1e-13, 0.01);

  EXPECT_NEAR(out.FrontWheelSteeringAngle[0], 0.0, 1e-6);
  EXPECT_NEAR(out.FrontWheelSteeringAngle[1], 0.0, 1e-6);
  EXPECT_NEAR(out.WheelRpm[0], out.WheelRpm[1], 1e-6);
  EXPECT_NEAR(out.WheelRpm[1], out.WheelRpm[2], 1e-6);
}

/**
 * @test NegativeVelocityClampedToMin
 * @brief Large negative command clamps to vmin; RPM magnitude matches clamped
 * value.
 */
TEST(AckermannTest, NegativeVelocityClampedToMin) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.6;
  cfg.DriveLength = 1.0;
  cfg.WheelRadius = 0.15;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-1.57, 1.57};

  AckermannController ctrl(cfg, make_vel_pid(), make_steer_pid());
  // Bootstrap:
  (void)ctrl.Compute(-20.0, 0.0, 0.00);
  auto out = ctrl.Compute(-20.0, 0.0, 0.01);

  EXPECT_NEAR(out.VelocityCommandApplied, -5.0, 1e-12);
  const double RAD_TO_RPM = 60.0 / (2.0 * M_PI);
  const double expected =
      (out.VelocityCommandApplied / cfg.WheelRadius) * RAD_TO_RPM;
  EXPECT_NEAR(out.WheelRpm[0], expected, 1e-3);
  EXPECT_TRUE(out.Clamped);
}

/* ----------------------- AckermannModel unit tests ----------------------- */

/**
 * @test AccelerationTest
 * @brief Speed integrates with acceleration; heading remains zero for straight
 * steering.
 *
 * @details
 * \f$v_{k+1} = v_k + a\Delta t\f$; with \f$v_0=1.0\f$, \f$a=2.0\f$, \f$\Delta
 * t=0.2\f$, expected \f$v=1.4\f$.
 */
TEST(AckermannModel, AccelerationTest) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.5;
  cfg.DriveLength = 1.2;
  cfg.WheelRadius = 0.2;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-1.0, 1.0};

  AckermannState initial_state{};
  initial_state.LongitudinalSpeed = 1.0;
  initial_state.HeadingAngle = 0.0;
  initial_state.SteeringAngle = 0.0;
  double delta_time = 0.2;
  AckermannModel model(cfg, initial_state, delta_time);

  AckermannState updated_state = model.Update(2.0, 0.0);
  double expected_speed = 1.0 + 2.0 * delta_time;
  EXPECT_NEAR(updated_state.LongitudinalSpeed, expected_speed, 1e-6);
  EXPECT_NEAR(updated_state.HeadingAngle, 0.0, 1e-6);
  EXPECT_NEAR(updated_state.SteeringAngle, 0.0, 1e-6);
}

/**
 * @test StraightMotion
 * @brief State remains unchanged with zero acceleration and zero steering.
 */
TEST(AckermannModel, StraightMotion) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.5;
  cfg.DriveLength = 1.2;
  cfg.WheelRadius = 0.2;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-1.0, 1.0};

  AckermannState initial_state{};
  initial_state.LongitudinalSpeed = 2.0;
  initial_state.HeadingAngle = 0.0;
  initial_state.SteeringAngle = 0.0;
  double delta_time = 0.1;
  AckermannModel model(cfg, initial_state, delta_time);
  AckermannState updated_state = model.Update(0.0, 0.0);
  EXPECT_NEAR(updated_state.LongitudinalSpeed, 2.0, 1e-6);
  EXPECT_NEAR(updated_state.HeadingAngle, 0.0, 1e-6);
  EXPECT_NEAR(updated_state.SteeringAngle, 0.0, 1e-6);
}

/**
 * @test TurningMotion
 * @brief Heading integrates according to bicycle model for non-zero steering.
 *
 * @details
 * \f$\psi_{k+1} = \psi_k + v\Delta t \tan(\delta)/L\f$.
 */
TEST(AckermannModel, TurningMotion) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.5;
  cfg.DriveLength = 1.2;
  cfg.WheelRadius = 0.2;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.LongitudinalSpeed = 3.0;
  initial_state.HeadingAngle = 0.0;
  initial_state.SteeringAngle = 0.0;
  double delta_time = 0.1;
  AckermannModel model(cfg, initial_state, delta_time);
  double steering_angle = 0.3;
  AckermannState updated_state = model.Update(0.0, steering_angle);

  double expected_heading_change =
      (3.0 * delta_time * std::tan(steering_angle)) / cfg.DriveLength;
  EXPECT_NEAR(updated_state.LongitudinalSpeed, 3.0, 1e-6);
  EXPECT_NEAR(updated_state.HeadingAngle, expected_heading_change, 1e-6);
  EXPECT_NEAR(updated_state.SteeringAngle, steering_angle, 1e-6);
}

/**
 * @test WheelSpeedsAndSteeringAnglesForTurning
 * @brief Non-zero steering produces non-zero steering angles and non-equal
 * RPMs.
 */
TEST(AckermannModel, WheelSpeedsAndSteeringAnglesForTurning) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.5;
  cfg.DriveLength = 1.2;
  cfg.WheelRadius = 0.2;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.LongitudinalSpeed = 4.0;
  initial_state.HeadingAngle = 0.0;
  initial_state.SteeringAngle = 0.2;
  double delta_time = 0.1;
  AckermannModel model(cfg, initial_state, delta_time);
  AckermannVehicleState vehicle_state = model.GetVehicleState();

  for (int i = 0; i < 4; ++i)
    EXPECT_GT(std::abs(vehicle_state.WheelRpm[i]), 0.0);
  for (int i = 0; i < 2; ++i)
    EXPECT_GT(std::abs(vehicle_state.WheelSteeringAngle[i]), 0.0);
}

/**
 * @test WheelSpeedsAndSteeringAnglesForStraight
 * @brief Straight steering yields equal RPMs and zero steering angles.
 */
TEST(AckermannModel, WheelSpeedsAndSteeringAnglesForStraight) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.5;
  cfg.DriveLength = 1.2;
  cfg.WheelRadius = 0.2;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.LongitudinalSpeed = 4.0;
  initial_state.HeadingAngle = 0.0;
  initial_state.SteeringAngle = 0.0;
  double delta_time = 0.1;
  AckermannModel model(cfg, initial_state, delta_time);
  AckermannVehicleState vehicle_state = model.GetVehicleState();

  EXPECT_NEAR(vehicle_state.WheelRpm[0], vehicle_state.WheelRpm[1], 1e-6);
  EXPECT_NEAR(vehicle_state.WheelRpm[1], vehicle_state.WheelRpm[2], 1e-6);
  EXPECT_NEAR(vehicle_state.WheelRpm[2], vehicle_state.WheelRpm[3], 1e-6);
  EXPECT_NEAR(vehicle_state.WheelSteeringAngle[0], 0.0, 1e-6);
  EXPECT_NEAR(vehicle_state.WheelSteeringAngle[1], 0.0, 1e-6);
}

/**
 * @test VelocityClamping
 * @brief Excess acceleration clamps speed to configured vmax.
 */
TEST(AckermannModel, VelocityClamping) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.5;
  cfg.DriveLength = 1.2;
  cfg.WheelRadius = 0.2;
  cfg.VelocityLimits = {-2.0, 2.0};
  cfg.SteeringLimits = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.LongitudinalSpeed = 1.5;
  initial_state.HeadingAngle = 0.0;
  initial_state.SteeringAngle = 0.0;
  double delta_time = 0.1;
  AckermannModel model(cfg, initial_state, delta_time);

  AckermannState updated_state = model.Update(10000.0, 0.0);
  EXPECT_NEAR(updated_state.LongitudinalSpeed, 2.0, 1e-6);
}

/**
 * @test SteeringClamping
 * @brief Steering angle clamps to configured bounds.
 */
TEST(AckermannModel, SteeringClamping) {
  AckermannConfig cfg{};
  cfg.DriveWidth = 0.5;
  cfg.DriveLength = 1.2;
  cfg.WheelRadius = 0.2;
  cfg.VelocityLimits = {-5.0, 5.0};
  cfg.SteeringLimits = {-0.3, 0.3};

  AckermannState initial_state{};
  initial_state.LongitudinalSpeed = 2.0;
  initial_state.HeadingAngle = 0.0;
  initial_state.SteeringAngle = 0.0;
  double delta_time = 0.1;
  AckermannModel model(cfg, initial_state, delta_time);

  AckermannState updated_state = model.Update(0.0, 1.0);
  EXPECT_NEAR(updated_state.SteeringAngle, 0.3, 1e-6);
}

/* --------------------------- PID unit tests --------------------------- */

/**
 * @test SetpointEqualsMeasurement
 * @brief Zero error yields zero output.
 *
 * @details
 * With \f$e=0\f$, P/I/D terms are all zero; output is 0 (within floating
 * tolerance).
 */
TEST(PIDControllerTest, SetpointEqualsMeasurement) {
  PIDController controller(1.0, 0.1, 0.01, 0.1, -100.0, 100.0);
  double output =
      controller.Compute(/*target_setpoint=*/5.0, /*measured_value=*/5.0);
  EXPECT_NEAR(output, 0.0, 1e-6);
}

/**
 * @test ProportionalOnlyBehavior
 * @brief With Ki=Kd=0, output equals Kp * error (within clamps).
 */
TEST(PIDControllerTest, ProportionalOnlyBehavior) {
  PIDController controller(/*Kp=*/2.0, /*Ki=*/0.0, /*Kd=*/0.0, 0.1, -10.0,
                           10.0);
  double output = controller.Compute(3.0, 1.0);
  EXPECT_NEAR(output, 4.0, 1e-6);
}

/**
 * @test ClampsToMaxOutput
 * @brief Large error saturates at max_output.
 */
TEST(PIDControllerTest, ClampsToMaxOutput) {
  PIDController controller(/*Kp=*/100.0, 0.0, 0.0, 0.1, -5.0, 5.0);
  double output = controller.Compute(10.0, 0.0);
  EXPECT_NEAR(output, 5.0, 1e-6);
}

/**
 * @test ClampsToMinOutput
 * @brief Large negative error saturates at min_output.
 */
TEST(PIDControllerTest, ClampsToMinOutput) {
  PIDController controller(/*Kp=*/100.0, 0.0, 0.0, 0.1, -5.0, 5.0);
  double output = controller.Compute(-10.0, 0.0);
  EXPECT_NEAR(output, -5.0, 1e-6);
}

/**
 * @test IntegralOnlyBehavior
 * @brief With Kp=Kd=0, output integrates error over time (within clamps).
 */
TEST(PIDControllerTest, IntegralOnlyBehavior) {
  const double Ki = 0.5, dt = 0.1;
  PIDController controller(0.0, Ki, 0.0, dt, -10.0, 10.0);
  double output1 = controller.Compute(5.0, 3.0);  // integral = 0.2, Iout = 0.1
  EXPECT_NEAR(output1, 0.1, 1e-6);
  double output2 = controller.Compute(5.0, 3.0);  // integral = 0.4, Iout = 0.2
  EXPECT_NEAR(output2, 0.2, 1e-6);
}

/**
 * @test DifferentialOnlyBehavior
 * @brief With Kp=Ki=0, output follows derivative of error.
 */
TEST(PIDControllerTest, DifferentialOnlyBehavior) {
  const double Kd = 0.5, dt = 0.1;
  PIDController controller(0.0, 0.0, Kd, dt, -10.0, 10.0);
  controller.SetDerivativeFilterTau(0.0);  // <-- add this

  double output1 = controller.Compute(5.0, 3.0);
  EXPECT_NEAR(output1, 10.0, 1e-6);
  double output2 = controller.Compute(5.0, 4.0);
  EXPECT_NEAR(output2, -5.0, 1e-6);
}

/**
 * @test CombinedPIDBehavior
 * @brief With all terms active, first step clamps, second step yields ~1.4.
 */
TEST(PIDControllerTest, CombinedPIDBehavior) {
  const double Kp = 1.0, Ki = 0.5, Kd = 0.1, dt = 0.1;
  PIDController controller(Kp, Ki, Kd, dt, -10.0, 10.0);
  controller.SetDerivativeFilterTau(0.0);  // <-- add this

  double output1 = controller.Compute(10.0, 5.0);
  EXPECT_NEAR(output1, 10.0, 1e-6);
  double output2 = controller.Compute(10.0, 7.0);
  EXPECT_NEAR(output2, 1.4, 1e-6);
}


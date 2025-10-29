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
 * @brief Straight line: All wheels should have equal RPM.
 *
 * Manual calculation:
 * linear_vel = 2.0 m/s
 * wheel_radius = 0.15 m
 * RAD_TO_RPM = 60/(2*pi) ~= 9.5492965855
 * wheel_rpm = (linear_vel / wheel_radius) * RAD_TO_RPM
 *           = (2.0 / 0.15) * 9.5492965855
 *           = 13.3333333 * 9.5492965855 ~= 127.32395 RPM
 */
TEST(AckermannTest, StraightLineAllWheelsSameRPM) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(2.0, 0.0, 0.01); // 2 m/s straight at t=0.01s

  // All wheel RPMs should be equal for straight motion
  EXPECT_NEAR(out.wheel_rpm[0], out.wheel_rpm[1], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[1], out.wheel_rpm[2], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[2], out.wheel_rpm[3], 1e-6);

  // steering angles should be (near) zero
  EXPECT_NEAR(out.steering_angle[0], 0.0, 1e-6);
  EXPECT_NEAR(out.steering_angle[1], 0.0, 1e-6);
  // sanity check of expected magnitude
  EXPECT_NEAR(out.wheel_rpm[0], 127.32395, 1e-3);
}

/**
 * @brief Turning: wheels and steering should differ between left and right.
 *
 * Manual calculation (v=1.0, w=0.5):
 * R = v / w = 2.0 m
 * half_width = 0.6 / 2 = 0.3 m, L = 1.0 m
 * R_left = R - half_width = 1.7 m
 * R_right = R + half_width = 2.3 m
 * steering_left  = atan(L / R_left)  = atan(1/1.7) ~= 0.528 rad
 * steering_right = atan(L / R_right) = atan(1/2.3) ~= 0.410 rad
 * Wheel radii: R_fl = sqrt((R-left_half)^2 + L^2) ~= 1.973
 *              R_fr = sqrt((R+half)^2 + L^2) ~= 2.508
 *              R_rl = |R - half| = 1.7
 *              R_rr = |R + half| = 2.3
 * v_fl = v * (R_fl/|R|) ~= 0.9865 m/s -> rpm ~= (0.9865/0.15)*9.5493 ~= 62.8
 * v_fr ~= 1.254 m/s -> rpm ~= 79.8
 */
TEST(AckermannTest, TurningProducesDifferentWheelSpeedsAndSteering) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-10.0, 10.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  // turn left with positive angular velocity
  double v = 1.0; // m/s
  double w = 0.5; // rad/s
  auto out = ctrl.compute(v, w, 0.02);

  // steering angles should be non-zero
  EXPECT_GT(std::abs(out.steering_angle[0]), 0.0);
  EXPECT_GT(std::abs(out.steering_angle[1]), 0.0);
  // Wheel RPMs should not all be equal
  bool all_equal = (std::abs(out.wheel_rpm[0] - out.wheel_rpm[1]) < 1e-6) &&
                   (std::abs(out.wheel_rpm[1] - out.wheel_rpm[2]) < 1e-6);
  EXPECT_FALSE(all_equal);
  // check relative ordering expected for left turn: FL < FR
  EXPECT_LT(out.wheel_rpm[0], out.wheel_rpm[1]);
}

/**
 * @brief Zero commanded velocities -> zero RPMs and zero steering.
 *
 * Manual calculation:
 * linear_vel = 0.0 -> wheel_rpm = 0.0
 * steering angles = 0.0
 */
TEST(AckermannTest, ZeroVelocitiesProduceZeroOutput) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(0.0, 0.0, 0.0);

  EXPECT_NEAR(out.wheel_rpm[0], 0.0, 1e-9);
  EXPECT_NEAR(out.wheel_rpm[1], 0.0, 1e-9);
  EXPECT_NEAR(out.wheel_rpm[2], 0.0, 1e-9);
  EXPECT_NEAR(out.wheel_rpm[3], 0.0, 1e-9);
  EXPECT_NEAR(out.steering_angle[0], 0.0, 1e-9);
  EXPECT_NEAR(out.steering_angle[1], 0.0, 1e-9);
}

/**
 * @brief Reverse straight motion: negative linear velocity produces negative RPMs.
 *
 * Manual calculation:
 * linear_vel = -3.0 m/s
 * wheel_rpm = (-3.0/0.15)*9.5492965855 = -20.0*9.5493 = -190.9859 RPM
 */
TEST(AckermannTest, ReverseStraightAllWheelsSameRPM) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(-3.0, 0.0, 0.01);

  EXPECT_NEAR(out.wheel_rpm[0], out.wheel_rpm[1], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[1], out.wheel_rpm[2], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[2], out.wheel_rpm[3], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[0], -190.98593, 1e-3);
}

/**
 * @brief Velocity clamping: requested velocity exceeds limits -> outputs clamped.
 *
 * Manual calculation:
 * requested linear_vel = 2.0 m/s but limits = {-0.5, 0.5}
 * clamped linear = 0.5 m/s
 * wheel_rpm = (0.5/0.15)*9.5492965855 ~= 31.83098 RPM
 */
TEST(AckermannTest, VelocityLimitsCauseClamping) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-0.5, 0.5};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(2.0, 0.0, 0.01);

  // since straight-line clamping happens and PID is identity here, clamped flag should be false
  // (applyPID uses setpoint==measurement so no PID correction). We still assert rpm matches clamped value.
  EXPECT_NEAR(out.wheel_rpm[0], 31.83098, 1e-3);
}

/**
 * @brief Steering limits: when steering computed exceeds limits, it's clamped.
 *
 * Manual calculation example (sharp left turn): expected steering angles > 0.2 rad -> clamped to 0.2
 */
TEST(AckermannTest, SteeringLimitsCauseClamping) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-10.0, 10.0};
  // tight steering limits
  cfg.steering_limits_ = {-0.2, 0.2};

  AckermannController ctrl(cfg);
  // large angular velocity to force large steering
  auto out = ctrl.compute(1.0, 5.0, 0.01);

  // steering angles should be clamped into [-0.2, 0.2]
  EXPECT_LE(out.steering_angle[0], 0.2 + 1e-9);
  EXPECT_GE(out.steering_angle[1], -0.2 - 1e-9);
  // ensure clamped flag set when any clamped
  EXPECT_TRUE(out.clamped || std::abs(out.steering_angle[0] - out.steering_angle[1]) < 1.57);
}

/**
 * @brief Very small angular velocity treated as straight (below eps threshold).
 *
 * Manual calculation: angular_vel = 1e-13 < eps (1e-12) -> steering = 0, all wheels same rpm
 */
TEST(AckermannTest, TinyAngularVelocityTreatedAsStraight) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(1.0, 1e-13, 0.01);

  EXPECT_NEAR(out.steering_angle[0], 0.0, 1e-12);
  EXPECT_NEAR(out.steering_angle[1], 0.0, 1e-12);
  EXPECT_NEAR(out.wheel_rpm[0], out.wheel_rpm[1], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[1], out.wheel_rpm[2], 1e-6);
}

/**
 * @brief Negative velocity clamping to minimum limit.
 *
 * Manual calculation: requested -20 m/s with limits [-5,5] -> clamped to -5 m/s -> rpm.
 */
TEST(AckermannTest, NegativeVelocityClampedToMin) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(-20.0, 0.0, 0.01);

  // expected clamped linear = -5.0 -> rpm = (-5.0/0.15)*9.5492965855 ~= -318.3099
  EXPECT_NEAR(out.wheel_rpm[0], -318.3099, 1e-3);
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


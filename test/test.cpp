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

/**
 * @brief Test AckermannModel update with acceleration input.
 * Manual calculation:
 * initial_speed = 1.0 m/s
 * acceleration = 2.0 m/s^2
 * delta_time = 0.2 s
 * expected_speed = initial_speed + acceleration * delta_time
 *                = 1.0 + 2.0 * 0.2 = 1.4 m/s
 */
TEST(AckermannModel, AccelerationTest) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.0, 1.0};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 1.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.2;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply acceleration and zero steering
  double acceleration = 2.0; // m/s^2
  AckermannState updated_state = model.update(acceleration, 0.0);
  // Expected speed: v + a * dt
  double expected_speed = 1.0 + acceleration * delta_time;
  EXPECT_NEAR(updated_state.longitudnal_speed_, expected_speed, 1e-6);
  EXPECT_NEAR(updated_state.heading_angle_, 0.0, 1e-6);
  EXPECT_NEAR(updated_state.steering_angle_, 0.0, 1e-6);
}

/**
 * @brief Test AckermannModel update with zero acceleration and steering for straight motion.
 */

TEST(AckermannModel, StraightMotion) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.0, 1.0};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 2.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply zero acceleration and zero steering for straight motion
  AckermannState updated_state = model.update(0.0, 0.0);
  EXPECT_NEAR(updated_state.longitudnal_speed_, 2.0, 1e-6);
  EXPECT_NEAR(updated_state.heading_angle_, 0.0, 1e-6);
  EXPECT_NEAR(updated_state.steering_angle_, 0.0, 1e-6);
}

/**
 * @brief Test AckermannModel update with zero acceleration and non-zero steering for turning motion.
 */

TEST(AckermannModel, TurningMotion) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 3.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply zero acceleration and a steering angle for turning
  double steering_angle = 0.3; // rad
  AckermannState updated_state = model.update(0.0, steering_angle);
  EXPECT_NEAR(updated_state.longitudnal_speed_, 3.0, 1e-6);
  // Expected heading change: v * dt * tan(steering) / L
  double expected_heading_change = (3.0 * delta_time * std::tan(steering_angle)) / cfg.drive_length_;
  EXPECT_NEAR(updated_state.heading_angle_, expected_heading_change, 1e-6);
  EXPECT_NEAR(updated_state.steering_angle_, steering_angle, 1e-6);
}

/**
 * @brief Test AckermannModel wheel speeds and steering angles computation for turning.
 */
TEST(AckermannModel, WheelSpeedsAndSteeringAnglesForTurning) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 4.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.2;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  AckermannVehicleState vehicle_state = model.get_vehicle_state();
  // Check that wheel RPMs and steering angles are computed (non-zero values)
  for (int i = 0; i < 4; ++i) {
    EXPECT_GT(std::abs(vehicle_state.wheel_rpm[i]), 0.0);
  }
  for (int i = 0; i < 2; ++i) {
    EXPECT_GT(std::abs(vehicle_state.wheel_steering_angle[i]), 0.0);
  }
}

/**
 * @brief Test AckermannModel wheel speeds and steering angles computation for straight motion.
 */

TEST(AckermannModel, WheelSpeedsAndSteeringAnglesForStraight) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 4.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  AckermannVehicleState vehicle_state = model.get_vehicle_state();
  // Check that all wheel RPMs are equal for straight motion
  EXPECT_NEAR(vehicle_state.wheel_rpm[0], vehicle_state.wheel_rpm[1], 1e-6);
  EXPECT_NEAR(vehicle_state.wheel_rpm[1], vehicle_state.wheel_rpm[2], 1e-6);
  EXPECT_NEAR(vehicle_state.wheel_rpm[2], vehicle_state.wheel_rpm[3], 1e-6);
  // Check that steering angles are zero for straight motion
  EXPECT_NEAR(vehicle_state.wheel_steering_angle[0], 0.0, 1e-6);
  EXPECT_NEAR(vehicle_state.wheel_steering_angle[1], 0.0, 1e-6);
}

/**
 * @brief Test AckermannModel velocity clamping when exceeding max limit.
 */

TEST(AckermannModel, VelocityClamping) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-2.0, 2.0};
  cfg.steering_limits_ = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 1.5; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply acceleration that would exceed max velocity
  double acceleration = 10000.0; // m/s^2
  AckermannState updated_state = model.update(acceleration, 0.0);
  // Speed should be clamped to max limit of 2.0 m/s
  EXPECT_NEAR(updated_state.longitudnal_speed_, 2.0, 1e-6);
}

/**
 * @brief Test AckermannModel steering angle clamping when exceeding limits.
 */
TEST(AckermannModel, SteeringClamping) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-0.3, 0.3};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 2.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply steering angle that exceeds limits
  double steering_angle = 1.0; // rad
  AckermannState updated_state = model.update(0.0, steering_angle);
  // Steering angle should be clamped to max limit of 0.3 rad
  EXPECT_NEAR(updated_state.steering_angle_, 0.3, 1e-6);
}


/**
 * @brief When setpoint equals measurement, controller should return zero value.
 * PID Gains: Kp = 1.0   Ki = 0.1   Kd = 0.01
 * delta_time = 0.1   min_output = -100.0   max_output = 100.0
 * setpoint = measurement = 5.0
 * error = setpoint - measurement = 5.0 - 5.0 = 0.0
 * Pout = 1.0 * 0.0 = 0.0
 * integral = 0.0 * 0.1 = 0.0
 * Iout = 0.1 * 0.0 = 0.0
 * derivative = (0.0 - 0.0) / 0.1 = 0.0
 * Dout = 0.01 * 0.0 = 0.0
 * expected_output = 0.0
 */
TEST(PIDControllerTest, SetpointEqualsMeasurement) {
  // Construct controller with example gains and limits.
  PIDController controller(1.0, 0.1, 0.01, 0.1, -100.0, 100.0);

  double setpoint = 5.0;
  double measurement = 5.0;  // same as setpoint
  double expected = 0.0;

  double output = controller.compute(setpoint, measurement);

  // Expect the controller output to be zero since error is zero
  // Use a small tolerance in case of floating-point computations.
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Proportional only (Ki, Kd = 0) control test
 * PID Gains: Kp = 2.0   Ki = 0.0   Kd = 0.0
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 * error = setpoint - meassurement = 3.0 - 1.0 = 2.0
 * Pout = 2.0 * 2.0 = 4.0
 * Iout = 0.0
 * Dout = 0.0
 * expected_output = 4.0
 * expected_output within clamp limits, so no clamps applied
 */
TEST(PIDControllerTest, ProportionalOnlyBehavior) {
  const double Kp = 2.0;
  // Ki=0 and Kd=0 to test proportional-only response
  PIDController controller(Kp, 0.0, 0.0, 0.1, -10.0, 10.0);

  double setpoint = 3.0;
  double measurement = 1.0;
  double expected = 4.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Controller output should be clamped to max_output.
 * PID Gains: Kp = 100.0   Ki = 0.0   Kd = 0.0
 * delta_time = 0.1   min_output = -5.0   max_output = 5.0
 * error = setpoint - measurement = 10.0 - 0.0 = 10.0
 * Pout = 100.0 * 10.0 = 1000.0
 * Iout = 0.0
 * Dout = 0.0
 * raw_output = 1000.0
 * expected_output = clamp(1000.0) = 5.0
 */
TEST(PIDControllerTest, ClampsToMaxOutput) {
  const double Kp = 100.0;
  PIDController controller(Kp, 0.0, 0.0, 0.1, -5.0, 5.0);

  double setpoint = 10.0;
  double measurement = 0.0;
  double expected = 5.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Controller output should be clamped to min_output.
 * PID Gains: Kp = 100.0   Ki = 0.0   Kd = 0.0
 * delta_time = 0.1   min_output = -5.0   max_output = 5.0
 * error = setpoint - measurement = -10.0 - 0.0 = -10.0
 * Pout = 100.0 * -10.0 = -1000.0
 * Iout = 0.0
 * Dout = 0.0
 * raw_output = -1000.0
 * expected_output = clamp(-1000.0) = -5.0
 */
TEST(PIDControllerTest, ClampsToMinOutput) {
  const double Kp = 100.0;
  PIDController controller(Kp, 0.0, 0.0, 0.1, -5.0, 5.0);

  double setpoint = -10.0;
  double measurement = 0.0;
  double expected = -5.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Integral only (Kp, Kd = 0) control test
 * PID Gains: Kp = 0.0   Ki = 0.5   Kd = 0.0
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 *
 * First step:
 * error = setpoint - measurement = 5.0 - 3.0 = 2.0
 * integral = prev_integral + (error * delta_time) = 0.0 + (2.0 * 0.1) = 0.2
 * expected_output1 = Ki * integral = 0.5 * 0.2 = 0.1
 *
 * Second step:
 * integral = previous_integral + (error * delta_time) = 0.2 + (2.0 * 0.1) = 0.4
 * expected_output2 = Ki * integral = 0.5 * 0.4 = 0.2
 */
TEST(PIDControllerTest, IntegralOnlyBehavior) {
  const double Ki = 0.5;
  const double dt = 0.1;
  PIDController controller(0.0, Ki, 0.0, dt, -10.0, 10.0);

  double setpoint = 5.0;
  double measurement = 3.0;
  double expected1 = 0.1;

  // First step
  double output1 = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output1, expected1, 1e-6);

  // Second step, integral should accumulate
  double expected2 = 0.2;
  double output2 = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output2, expected2, 1e-6);
}

/**
 * @brief Differential only (Kp, Ki = 0) control test
 * PID Gains: Kp = 0.0   Ki = 0.0   Kd = 0.5
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 *
 * First step:
 * error1 = setpoint - measurement1 = 5.0 - 3.0 = 2.0
 * previous_error = 0.0 (initial)
 * derivative = (error1 - previous_error) / dt = (2.0 - 0.0) / 0.1 = 20.0
 * expected_output1 = Kd * derivative = 0.5 * 20.0 = 10.0
 *
 * Second step:
 * error2 = setpoint - measurement2 = 5.0 - 4.0 = 1.0
 * derivative = (error2 - error1) / dt = (1.0 - 2.0) / 0.1 = -10.0
 * expected_output2 = Kd * derivative = 0.5 * (-10.0) = -5.0
 */
TEST(PIDControllerTest, DifferentialOnlyBehavior) {
  const double Kd = 0.5;
  const double dt = 0.1;
  PIDController controller(0.0, 0.0, Kd, dt, -10.0, 10.0);

  double setpoint = 5.0;
  double measurement1 = 3.0;
  double expected1 = 10.0;

  // First step
  double output1 = controller.compute(setpoint, measurement1);
  EXPECT_NEAR(output1, expected1, 1e-6);

  // Second step
  double measurement2 = 4.0;
  double expected2 = -5.0;
  double output2 = controller.compute(setpoint, measurement2);
  EXPECT_NEAR(output2, expected2, 1e-6);
}

/**
 * @brief Test with all PID terms active over two iterations
 * PID Gains: Kp = 1.0   Ki = 0.5   Kd = 0.1
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 * setpoint = 10.0
 *
 * First iteration (measurement = 5.0):
 * error1 = 10.0 - 5.0 = 5.0
 * Pout1 = 1.0 * 5.0 = 5.0
 * integral1 = 0.0 + 5.0 * 0.1 = 0.5
 * Iout1 = 0.5 * 0.5 = 0.25
 * derivative1 = (5.0 - 0.0) / 0.1 = 50.0
 * Dout1 = 0.1 * 50.0 = 5.0
 * output1 = 5.0 + 0.25 + 5.0 = 10.25
 * clamped_output1 = 10.0
 *
 * Second iteration (measurement = 7.0):
 * error2 = 10.0 - 7.0 = 3.0
 * Pout2 = 1.0 * 3.0 = 3.0
 * integral2 = 0.5 + 3.0 * 0.1 = 0.8
 * Iout2 = 0.5 * 0.8 = 0.4
 * derivative2 = (3.0 - 5.0) / 0.1 = -20.0
 * Dout2 = 0.1 * -20.0 = -2.0
 * output2 = 3.0 + 0.4 - 2.0 = 1.4
 */
TEST(PIDControllerTest, CombinedPIDBehavior) {
  const double Kp = 1.0;
  const double Ki = 0.5;
  const double Kd = 0.1;
  const double dt = 0.1;
  PIDController controller(Kp, Ki, Kd, dt, -10.0, 10.0);

  double setpoint = 10.0;

  // First iteration
  double measurement1 = 5.0;
  double expected1 = 10.0;  // Would be 10.25 but clamped to 10.0
  double output1 = controller.compute(setpoint, measurement1);
  EXPECT_NEAR(output1, expected1, 1e-6);

  // Second iteration
  double measurement2 = 7.0;
  double expected2 = 1.4;
}
